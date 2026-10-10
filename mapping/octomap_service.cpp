#include <octomap/OcTree.h>
#include <nlohmann/json.hpp>
#include <openssl/evp.h>
#include <algorithm>
#include <array>
#include <cmath>
#include <deque>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <iomanip>
#include <limits>
#include <map>
#include <memory>
#include <set>
#include <stdexcept>
#include <string>
#include <sstream>
#include <vector>
#include <unordered_set>

using json = nlohmann::json;
using Key = std::array<int, 3>;
using Vec = std::array<double, 3>;
// Full-depth OcTree keys occupy 48 bits. Mix the packed coordinates to avoid
// poor bucket distribution along long axis-aligned rays.
struct KeyHash {
  size_t operator()(const Key& k) const noexcept {
    uint64_t x=uint64_t(k[0]+32768) | (uint64_t(k[1]+32768)<<16) | (uint64_t(k[2]+32768)<<32);
    x^=x>>30; x*=0xbf58476d1ce4e5b9ULL;
    x^=x>>27; x*=0x94d049bb133111ebULL;
    return size_t(x^(x>>31));
  }
};

struct Error : std::runtime_error {
  std::string code;
  Error(std::string c, std::string m) : std::runtime_error(m), code(std::move(c)) {}
};
static void require(bool ok, const std::string& code, const std::string& message) {
  if (!ok) throw Error(code, message);
}
static Vec vector3(const json& value) {
  require(value.is_array() && value.size() == 3, "invalid_request", "Expected three coordinates");
  Vec result{};
  for (int i = 0; i < 3; ++i) {
    require(value[i].is_number(), "invalid_request", "Coordinates must be numeric");
    result[i] = value[i].get<double>();
    require(std::isfinite(result[i]), "invalid_request", "Coordinates must be finite");
  }
  return result;
}
static double finite_number(const json& request, const char* name) {
  const double value = request.at(name).get<double>();
  require(std::isfinite(value), "invalid_request", std::string(name) + " must be finite");
  return value;
}
static std::string identifier(const json& request, const char* name) {
  const auto value = request.at(name).get<std::string>();
  require(!value.empty() && value.size() <= 256, "invalid_request", std::string(name) + " must be a nonempty identifier");
  return value;
}
static octomap::point3d point(const Vec& p) { return {float(p[0]), float(p[1]), float(p[2])}; }
static std::string sha256_file(const std::string& path) {
  std::ifstream input(path,std::ios::binary);
  require(bool(input),"io_error","Cannot hash map file");
  std::unique_ptr<EVP_MD_CTX,decltype(&EVP_MD_CTX_free)> context(EVP_MD_CTX_new(),EVP_MD_CTX_free);
  require(context && EVP_DigestInit_ex(context.get(),EVP_sha256(),nullptr)==1,"io_error","Cannot initialize checksum");
  char buffer[65536];
  while(input.read(buffer,sizeof(buffer)) || input.gcount()) {
    require(EVP_DigestUpdate(context.get(),buffer,input.gcount())==1,"io_error","Cannot update checksum");
  }
  require(input.eof(),"io_error","Map read failed while hashing");
  unsigned char digest[EVP_MAX_MD_SIZE]; unsigned int length=0;
  require(EVP_DigestFinal_ex(context.get(),digest,&length)==1,"io_error","Cannot finalize checksum");
  std::ostringstream output;
  for(unsigned int i=0;i<length;++i) output<<std::hex<<std::setw(2)<<std::setfill('0')<<int(digest[i]);
  return output.str();
}
static Key signed_key(const octomap::OcTreeKey& k) { return {int(k[0])-32768, int(k[1])-32768, int(k[2])-32768}; }
static octomap::OcTreeKey octo_key(const Key& k) {
  for (int a : k) require(a >= -32768 && a < 32768, "invalid_request", "Key outside OctoMap range");
  return {octomap::key_type(k[0]+32768), octomap::key_type(k[1]+32768), octomap::key_type(k[2]+32768)};
}

class Service {
  struct Stream { uint64_t maximum = 0; std::set<uint64_t> seen; };
  struct Revision { uint64_t number; std::set<Key> touched, classified; };
  std::unique_ptr<octomap::OcTree> tree;
  json config;
  uint64_t revision = 0;
  double watermark = -1;
  // Only observation/marking provenance is indexed here. Occupancy stays in OcTree.
  std::map<Key, double> observed;
  // Marking provenance, separate from free-space observation/freshness. A real
  // far return may update one of these existing surfaces even after a miss has
  // changed it to free, but cannot initialize previously unmarked far space.
  std::set<Key> known_near_hits;
  std::map<std::pair<std::string,std::string>, Stream> streams;
  std::deque<Revision> history;
  size_t history_limit = 128;
  uint64_t sequence_window = 4096;
  double reorder_window = 0.25;
  Vec bounds_min{}, bounds_max{};
  bool bounded = false;

  json envelope() const {
    return {{"schema","air_ground_octomap/v1"},{"scene_id",config.at("scene_id")},
      {"scene_epoch",config.at("scene_epoch")},{"content_version",config.at("content_version")},
      {"frame_id",config.at("frame_id")},{"resolution",tree->getResolution()},
      {"revision",revision},{"capture_sim_time_watermark",watermark}};
  }
  bool inside(const Vec& p) const {
    if (!bounded) return true;
    for (int a=0; a<3; ++a) if (p[a]<bounds_min[a] || p[a]>=bounds_max[a]) return false;
    return true;
  }
  Key key_for(const Vec& p) const {
    octomap::OcTreeKey k(0,0,0);
    require(tree->coordToKeyChecked(p[0],p[1],p[2],k), "out_of_bounds", "Coordinate outside OctoMap range");
    return signed_key(k);
  }
  json cell(const Key& key) const {
    const auto* node = tree->search(octo_key(key));
    json result={{"key",key},{"state","unknown"},{"log_odds",nullptr},{"last_observed_sim_time",nullptr}};
    if (node) {
      result["state"]=tree->isNodeOccupied(node) ? "occupied" : "free";
      result["log_odds"]=node->getLogOdds();
      auto stamp=observed.find(key);
      if (stamp!=observed.end()) result["last_observed_sim_time"]=stamp->second;
    }
    return result;
  }
  int state(const Key& key) const {
    const auto* node=tree->search(octo_key(key));
    return node ? (tree->isNodeOccupied(node) ? 2 : 1) : 0;
  }
  static bool binary_refresh(const json& req) {
    const auto encoding=req.value("refresh_encoding",std::string("keys"));
    require(encoding=="keys" || encoding=="i16le_base64","invalid_refresh_encoding","Expected keys or i16le_base64");
    const bool binary=encoding=="i16le_base64";
    require(!binary || req.value("compact",false),"invalid_refresh_encoding","Binary refresh keys require compact=true");
    return binary;
  }
  static std::string encode_keys_i16le(const std::vector<Key>& keys) {
    require(keys.size()<=2000000,"response_too_large","Binary refresh limit is 2000000 keys");
    if(keys.empty()) return "";
    std::vector<unsigned char> raw(keys.size()*6);
    size_t position=0;
    for(const auto& key:keys) for(int value:key) {
      // Signed OctoMap keys are exactly the uint16 OcTreeKey minus 32768.
      require(value>=-32768 && value<32768,"invalid_key","OctoMap key outside signed16 range");
      const auto bits=static_cast<uint16_t>(value);
      raw[position++]=static_cast<unsigned char>(bits&255u);
      raw[position++]=static_cast<unsigned char>(bits>>8);
    }
    std::string encoded(4*((raw.size()+2)/3)+1,'\0');
    const int length=EVP_EncodeBlock(reinterpret_cast<unsigned char*>(encoded.data()),raw.data(),static_cast<int>(raw.size()));
    require(length>=0,"encoding_error","Cannot encode refresh keys");
    encoded.resize(static_cast<size_t>(length));
    return encoded;
  }
  json delta_response(uint64_t base,const std::set<Key>& touched,const std::set<Key>& classified,bool compact,bool presence_only=false,bool binary=false) const {
    auto response=envelope(); response["base_revision"]=base;
    if(presence_only) response["refresh_semantics"]="presence_only";
    if(binary) response["refresh_encoding"]="i16le_base64";
    response["changed_cells"]=json::array();
    for(const auto& key:(compact ? classified : touched)) response["changed_cells"].push_back(cell(key));
    if(compact) {
      response["compact"]=true;
      response["refreshes"]=json::array();
      if(!presence_only) {
        std::map<double,std::vector<Key>> stamps;
        for(const auto& key:touched) if(!classified.count(key)) {
          auto it=observed.find(key);
          if(it!=observed.end()) stamps[it->second].push_back(key);
        }
        for(const auto& row:stamps) {
          if(binary) response["refreshes"].push_back({{"capture_sim_time",row.first},
            {"key_count",row.second.size()},{"keys_i16le_b64",encode_keys_i16le(row.second)}});
          else response["refreshes"].push_back({{"capture_sim_time",row.first},{"keys",row.second}});
        }
      }
    }
    return response;
  }
  json commit(const std::set<Key>& keys,const std::set<Key>& classified,bool compact,bool binary=false) {
    const uint64_t base=revision;
    if (!keys.empty()) {
      ++revision;
      history.push_back({revision,keys,classified});
      while(history.size()>history_limit) history.pop_front();
    }
    return delta_response(base,keys,classified,compact,false,binary);
  }
  void check_identity(const json& req) const {
    require(req.at("scene_epoch")==config.at("scene_epoch") && req.at("scene_id")==config.at("scene_id"),
      "scene_mismatch", "Observation belongs to another scene or episode");
  }
  void initialize(const json& req) {
    require(!tree, "already_initialized", "Use a new process for a new scene epoch");
    for (auto name:{"scene_id","scene_epoch","content_version","frame_id"}) identifier(req,name);
    const double resolution=req.value("resolution",0.25);
    require(std::isfinite(resolution) && resolution>=0.01 && resolution<=5.0,"invalid_request","Invalid resolution");
    const auto new_reorder=req.value("out_of_order_window_s",0.25);
    require(std::isfinite(new_reorder) && new_reorder>=0 && new_reorder<=10,"invalid_request","Invalid reorder window");
    const auto new_history=req.value("history_limit",128);
    require(new_history>=1 && new_history<=10000,"invalid_request","Invalid history limit");
    bool new_bounded=req.contains("bounds"); Vec lo{},hi{};
    if(new_bounded) {
      lo=vector3(req.at("bounds").at("min")); hi=vector3(req.at("bounds").at("max"));
      for(int a=0;a<3;++a) require(lo[a]<hi[a],"invalid_request","Invalid map bounds");
    }
    auto candidate=std::make_unique<octomap::OcTree>(resolution);
    const double hit=req.value("prob_hit",0.7), miss=req.value("prob_miss",0.4);
    const double min=req.value("clamp_min",0.12), max=req.value("clamp_max",0.97);
    require(hit>0.5 && hit<1 && miss>0 && miss<0.5 && min>0 && min<0.5 && max>0.5 && max<1,
      "invalid_request","Invalid probability model");
    candidate->setProbHit(hit); candidate->setProbMiss(miss);
    candidate->setClampingThresMin(min); candidate->setClampingThresMax(max);
    config=req; config["resolution"]=resolution;
    tree=std::move(candidate); reorder_window=new_reorder; history_limit=new_history;
    bounded=new_bounded; bounds_min=lo; bounds_max=hi;
  }
  json integrate(const json& req) {
    const bool binary=binary_refresh(req); // Validate before any observation or stream mutation.
    const bool compact=req.value("compact",false);
    check_identity(req);
    const auto robot=identifier(req,"robot_id"), sensor=identifier(req,"sensor_id");
    identifier(req,"calibration_id");
    require(req.at("frame_seq").is_number_unsigned() || (req.at("frame_seq").is_number_integer() && req.at("frame_seq").get<int64_t>()>=0),
      "invalid_request","frame_seq must be a nonnegative integer");
    const auto seq=req.at("frame_seq").get<uint64_t>();
    const double stamp=finite_number(req,"capture_sim_time");
    finite_number(req,"receive_wall_time");
    require(stamp>=0,"invalid_request","capture_sim_time must be nonnegative");
    auto existing=streams.find({robot,sensor});
    if(existing!=streams.end()) {
      if(existing->second.seen.count(seq)) { auto out=delta_response(revision,{}, {},req.value("compact",false),false,binary); out["duplicate"]=true; return out; }
      require(seq>=existing->second.maximum || existing->second.maximum-seq<=sequence_window,
        "sequence_too_old","Observation sequence is outside deduplication retention window");
    }
    require(watermark<0 || stamp>=watermark-reorder_window,"observation_too_old","Capture time is outside allowed reordering window");
    const Vec origin=vector3(req.at("origin"));
    require(inside(origin),"out_of_bounds","Capture origin is outside map bounds");
    key_for(origin);
    const double max_range=req.value("max_range_m",12.0), min_range=req.value("min_range_m",0.05);
    require(std::isfinite(max_range) && max_range>0 && max_range<=1000 && std::isfinite(min_range) && min_range>=0 && min_range<max_range,
      "invalid_request","Invalid observation range");
    const double mark_range=req.value("mark_range_m",max_range);
    require(std::isfinite(mark_range) && mark_range>0 && mark_range<=max_range,
      "invalid_request","Invalid mark_range_m");
    const auto points=req.value("points",json::array());
    require(points.is_array() && points.size()<=250000,"invalid_request","points must be an array of at most 250000 hits");
    // Validate and rasterize the complete frame before mutating the authoritative tree.
    std::set<Key> hits;
    // Rays repeatedly cross the same nearby voxels. Hash deduplication avoids
    // an ordered-tree lookup for every traversed voxel. Published revisions
    // still use sorted keys; occupancy/probability semantics are unchanged.
    std::unordered_set<Key,KeyHash> frees;
    frees.reserve(16384);
    size_t discarded=0;
    octomap::KeyRay ray;
    for(const auto& item:points) {
      const Vec end=vector3(item);
      double squared=0; for(int a=0;a<3;++a) squared+=(end[a]-origin[a])*(end[a]-origin[a]);
      const double distance=std::sqrt(squared);
      if(distance<min_range || distance>mark_range || !inside(end)) { ++discarded; continue; }
      const Key hit=key_for(end);
      require(tree->computeRayKeys(point(origin),point(end),ray),"ray_error","Cannot rasterize capture ray");
      hits.insert(hit);
      for(const auto& k:ray) frees.insert(signed_key(k));
    }
    const auto free_rays=req.value("free_rays",json::array());
    require(free_rays.is_array() && free_rays.size()<=250000,"invalid_request","Invalid free_rays");
    for(const auto& item:free_rays) {
      const auto source=item.at("source").get<std::string>();
      require(source=="registered_depth_measured_beyond_range" || source=="carla_depth_far_plane" ||
        source=="lidar_measured_beyond_range",
        "invalid_free_ray_source","Free-only rays require explicit measured-hit or simulator far-plane evidence");
      const Vec end=vector3(item.at("endpoint"));
      double squared=0; for(int a=0;a<3;++a) squared+=(end[a]-origin[a])*(end[a]-origin[a]);
      const double distance=std::sqrt(squared);
      if(distance<min_range || distance>max_range+1e-6 || !inside(end)) { ++discarded; continue; }
      const Key end_key=key_for(end);
      require(tree->computeRayKeys(point(origin),point(end),ray),"ray_error","Cannot rasterize free-only ray");
      for(const auto& k:ray) frees.insert(signed_key(k));
      // The real measured surface is not free evidence. A caller marking
      // only near hits can clear the prefix of farther measured rays while
      // excluding their endpoint. Cropped free rays retain the old default.
      if(item.value("include_endpoint",true)) frees.insert(end_key);
    }
    // Real far returns do not create occupancy outside marking range, but
    // their measured surfaces cannot be erased by *another* ray in this frame.
    // Validate all protection evidence before mutating occupancy or metadata.
    const auto protected_points=req.value("protected_hit_points",json::array());
    require(protected_points.is_array() && protected_points.size()<=250000,
      "invalid_request","protected_hit_points must be an array of at most 250000 real hits");
    std::set<Key> protected_hits;
    if(!protected_points.empty()) {
      require(req.contains("mark_range_m"),"invalid_request","Protected far hits require mark_range_m");
      for(const auto& item:protected_points) {
        const Vec end=vector3(item);
        double squared=0; for(int a=0;a<3;++a) squared+=(end[a]-origin[a])*(end[a]-origin[a]);
        const double distance=std::sqrt(squared);
        require(std::isfinite(distance) && distance>mark_range && distance<=max_range &&
          distance>=min_range && inside(end),"invalid_request","Protected point must be a real far hit within clearing range and bounds");
        protected_hits.insert(key_for(end));
      }
    }
    // Near hits and real far surfaces win across the whole observation.
    for(const auto& k:hits) frees.erase(k);
    for(const auto& k:protected_hits) frees.erase(k);
    // Bound optional binary decoding before mutating occupancy/metadata.
    require(!binary || hits.size()+frees.size()+protected_hits.size()<=2000000,"response_too_large","Binary observation exceeds 2000000 touched keys");
    // Arrival order is not measurement order across robots/cameras. Older
    // evidence must not reverse a newer state. At an equal capture time a hit
    // takes precedence over a free ray, including across separate sensors.
    std::set<Key> touched, classified;
    std::vector<octomap::OcTreeKey> probability_dirty;
    size_t skipped_stale=0;
    // Keep updates spatially ordered for OcTree/cache locality after fast
    // deduplication. Only unique keys are copied, never all ray steps.
    std::vector<Key> ordered_frees(frees.begin(),frees.end());
    std::sort(ordered_frees.begin(),ordered_frees.end());
    for(const auto& k:ordered_frees) {
      const auto previous=observed.find(k);
      if(previous!=observed.end() && (stamp<previous->second-1e-9 ||
          (std::abs(stamp-previous->second)<=1e-9 && state(k)==2))) { ++skipped_stale; continue; }
      const auto key=octo_key(k);
      const auto* before_node=tree->search(key);
      const int before=before_node ? (tree->isNodeOccupied(before_node)?2:1) : 0;
      const float before_log=before_node ? before_node->getLogOdds() : 0.f;
      // Same saturated-leaf early return as installed OctoMap; reuse the
      // already resolved leaf instead of performing a second root lookup.
      const auto* after=(before_node && before_log <= tree->getClampingThresMinLog())
        ? before_node : tree->updateNode(key,false,true);
      if(!before_node || before_log!=after->getLogOdds()) probability_dirty.push_back(key);
      // Restored cells retain occupancy but have no current-epoch evidence.
      // Their first accepted observation is a semantic change even when the
      // occupancy class remains unchanged. Planners must learn None -> Some.
      const bool first_observation=previous==observed.end();
      touched.insert(k);
      if(first_observation) observed.emplace(k,stamp); else previous->second=stamp;
      if(first_observation || before!=(tree->isNodeOccupied(after)?2:1)) classified.insert(k);
    }
    // Far positive evidence is symmetric with far free evidence for surfaces
    // that were actually marked in range. Do not use observed: it also indexes
    // cells that have only ever been traversed by free rays.
    std::set<Key> positive_hits=hits;
    for(const auto& k:protected_hits) if(known_near_hits.count(k)) positive_hits.insert(k);
    size_t refreshed_protected_hits=0, recovered_far_hit_cells=0;
    for(const auto& k:positive_hits) {
      const auto previous=observed.find(k);
      if(previous!=observed.end() && stamp<previous->second-1e-9) { ++skipped_stale; continue; }
      const auto key=octo_key(k);
      const auto* before_node=tree->search(key);
      const int before=before_node ? (tree->isNodeOccupied(before_node)?2:1) : 0;
      const float before_log=before_node ? before_node->getLogOdds() : 0.f;
      // Same saturated-leaf early return as installed OctoMap; reuse the
      // already resolved leaf instead of performing a second root lookup.
      const auto* after=(before_node && before_log >= tree->getClampingThresMaxLog())
        ? before_node : tree->updateNode(key,true,true);
      if(!before_node || before_log!=after->getLogOdds()) probability_dirty.push_back(key);
      // Restored cells retain occupancy but have no current-epoch evidence.
      // Their first accepted observation is a semantic change even when the
      // occupancy class remains unchanged. Planners must learn None -> Some.
      const bool first_observation=previous==observed.end();
      touched.insert(k);
      if(first_observation) observed.emplace(k,stamp); else previous->second=stamp;
      if(first_observation || before!=(tree->isNodeOccupied(after)?2:1)) classified.insert(k);
      if(hits.count(k)) {
        known_near_hits.insert(k);
      } else {
        ++refreshed_protected_hits;
        if(before!=2 && tree->isNodeOccupied(after)) ++recovered_far_hit_cells;
      }
    }
    if(probability_dirty.size()>4096) {
      tree->updateInnerOccupancy();
    } else if(!probability_dirty.empty()) {
      // Only ancestors of leaves whose actual probability changed can have a
      // different max-child probability. Keep every fixed-depth leaf; no prune.
      std::array<std::unordered_set<octomap::OcTreeNode*>,16> ancestors;
      for(const auto& key:probability_dirty) {
        auto* node=tree->getRoot();
        for(unsigned depth=0;depth<16 && node;++depth) {
          ancestors[depth].insert(node);
          const unsigned child=octomap::computeChildIdx(key,15-depth);
          node=tree->nodeChildExists(node,child)?tree->getNodeChild(node,child):nullptr;
        }
      }
      for(int depth=15;depth>=0;--depth)
        for(auto* node:ancestors[depth]) node->updateOccupancyChildren();
    }
    auto& stream=streams[{robot,sensor}]; stream.maximum=std::max(stream.maximum,seq); stream.seen.insert(seq);
    while(!stream.seen.empty() && *stream.seen.begin()<stream.maximum && stream.maximum-*stream.seen.begin()>sequence_window) stream.seen.erase(stream.seen.begin());
    watermark=std::max(watermark,stamp);
    auto out=commit(touched,classified,compact,binary); out["duplicate"]=false; out["accepted_hits"]=hits.size();
    out["free_cells"]=frees.size(); out["discarded_points"]=discarded; out["skipped_stale_cells"]=skipped_stale;
    out["protected_hit_cells"]=protected_hits.size();
    out["refreshed_protected_hits"]=refreshed_protected_hits;
    out["recovered_far_hit_cells"]=recovered_far_hit_cells;
    return out;
  }
  json snapshot() const {
    auto out=envelope(); out["cells"]=json::array();
    // No pruning is performed: leaf cells have the configured resolution.
    for(auto it=tree->begin_leafs();it!=tree->end_leafs();++it) out["cells"].push_back(cell(signed_key(it.getKey())));
    return out;
  }
  json changes(const json& req) const {
    const bool binary=binary_refresh(req);
    const auto semantics=req.value("refresh_semantics",std::string("full"));
    require(semantics=="full" || semantics=="presence_only","invalid_refresh_semantics","Expected full or presence_only");
    const bool presence_only=semantics=="presence_only";
    const bool compact=req.value("compact",false);
    require(!presence_only || compact,"invalid_refresh_semantics","presence_only requires compact=true");
    const auto since=req.at("since_revision").get<uint64_t>();
    require(since<=revision,"future_revision","Requested revision is newer than this map");
    require(since==revision || (!history.empty() && since>=history.front().number-1),"snapshot_required","Requested delta history has expired; fetch snapshot");
    std::set<Key> touched,classified;
    for(const auto& entry:history) if(entry.number>since) {
      // The planner view needs occupancy and first current-epoch observation
      // only. Do not even build the much larger repeated-freshness key union.
      if(!presence_only) touched.insert(entry.touched.begin(),entry.touched.end());
      classified.insert(entry.classified.begin(),entry.classified.end());
    }
    require(!binary || touched.size()<=2000000,"response_too_large","Binary delta exceeds 2000000 touched keys");
    return delta_response(since,touched,classified,compact,presence_only,binary);
  }
  json forget(const json& req) {
    const bool binary=binary_refresh(req);
    check_identity(req);
    require(req.value("reason",std::string()).size()>0,"invalid_request","Explicit forgetting requires a reason");
    const auto& raw=req.at("keys"); require(raw.is_array() && raw.size()<=250000,"invalid_request","Invalid keys");
    std::set<Key> keys;
    for(const auto& item:raw) { const Key key=item.get<Key>(); octo_key(key); keys.insert(key); }
    std::set<Key> changed;
    for(const auto& key:keys) {
      known_near_hits.erase(key);
      if(tree->search(octo_key(key))) { tree->deleteNode(octo_key(key)); observed.erase(key); changed.insert(key); }
    }
    tree->updateInnerOccupancy(); return commit(changed,changed,req.value("compact",false),binary);
  }
  json save(const json& req) const {
    const std::filesystem::path path=req.at("path").get<std::string>();
    require(path.extension()==".ot","invalid_request","Full-probability map requires an .ot path");
    const auto temporary=path.string()+".tmp";
    require(tree->write(temporary),"io_error","Could not write full OctoMap");
    json meta=envelope(); meta["config"]=config; meta["observations"]=json::array();
    for(const auto& item:observed) meta["observations"].push_back({{"key",item.first},{"capture_sim_time",item.second}});
    meta["known_near_hit_keys"]=known_near_hits;
    meta["format"]="OctoMap full probability .ot";
    meta["ot_sha256"]=sha256_file(temporary);
    std::ofstream stream(path.string()+".metadata.json.tmp"); stream<<meta.dump(2)<<'\n'; stream.close();
    require(bool(stream),"io_error","Could not write map metadata");
    std::filesystem::rename(temporary,path);
    std::filesystem::rename(path.string()+".metadata.json.tmp",path.string()+".metadata.json");
    auto out=envelope(); out["path"]=path.string(); out["metadata_path"]=path.string()+".metadata.json"; return out;
  }
  json load(const json& req) {
    require(revision==0 && tree->size()==0 && streams.empty(),"map_not_empty","Restore is only allowed before observations");
    const std::string path=req.at("path").get<std::string>();
    std::ifstream stream(path+".metadata.json"); require(bool(stream),"io_error","Missing map metadata");
    json meta; stream>>meta;
    require(meta.at("ot_sha256").get<std::string>()==sha256_file(path),"checksum_mismatch","Map and metadata do not match");
    for(auto name:{"scene_id","content_version","frame_id"}) require(meta.at(name)==config.at(name),"restore_mismatch",std::string("Snapshot identity mismatch: ")+name);
    require(meta.at("resolution").get<double>()==tree->getResolution(),"restore_mismatch","Snapshot resolution mismatch");
    std::unique_ptr<octomap::AbstractOcTree> loaded(octomap::AbstractOcTree::read(path));
    require(bool(loaded),"io_error","Could not read .ot map");
    auto* typed=dynamic_cast<octomap::OcTree*>(loaded.get());
    require(typed && typed->getResolution()==tree->getResolution(),"restore_mismatch","Expected matching OcTree");
    // Restore retains probability and current input model, but requires a fresh episode.
    require(meta.at("scene_epoch")!=config.at("scene_epoch"),"restore_mismatch","Restore must use a fresh scene_epoch");
    std::map<Key,double> stamps;
    for(const auto& row:meta.at("observations")) { auto k=row.at("key").get<Key>(); octo_key(k); stamps[k]=row.at("capture_sim_time").get<double>(); }
    std::set<Key> restored_known_near_hits;
    if(meta.contains("known_near_hit_keys")) {
      const auto& saved=meta.at("known_near_hit_keys");
      require(saved.is_array(),"restore_mismatch","Known surface history must be an array");
      for(const auto& row:saved) {
        require(row.is_array() && row.size()==3,"restore_mismatch","Known surface key must have three integers");
        Key k{};
        for(size_t i=0;i<3;++i) {
          require(row[i].is_number_integer() && row[i]>=-32768 && row[i]<32768,
            "restore_mismatch","Known surface key is outside signed OctoMap range");
          k[i]=row[i].get<int>();
        }
        require(typed->search(octo_key(k)),"restore_mismatch","Known surface history references a missing map cell");
        restored_known_near_hits.insert(k);
      }
    } else {
      // Older snapshots did not save marking provenance. Only occupied leaves
      // certify a past hit; observed/free cells do not. Cleared legacy surfaces
      // need a new in-range hit before far returns can restore them.
      for(auto it=typed->begin_leafs();it!=typed->end_leafs();++it)
        if(typed->isNodeOccupied(*it)) restored_known_near_hits.insert(signed_key(it.getKey()));
    }
    typed->setProbHit(tree->getProbHit()); typed->setProbMiss(tree->getProbMiss());
    typed->setClampingThresMin(tree->getClampingThresMin()); typed->setClampingThresMax(tree->getClampingThresMax());
    tree.reset(static_cast<octomap::OcTree*>(loaded.release()));
    observed=std::move(stamps);
    known_near_hits=std::move(restored_known_near_hits);
    // Old episode timestamps cannot certify freshness in a new episode.
    observed.clear();
    revision=1; history.clear(); watermark=-1;
    auto out=snapshot(); out["restored_from_epoch"]=meta.at("scene_epoch"); out["base_revision"]=0; return out;
  }
public:
  json dispatch(const json& req) {
    const auto op=req.at("op").get<std::string>();
    if(op=="init") { initialize(req); return envelope(); }
    require(bool(tree),"not_initialized","Call init first");
    if(op=="integrate") return integrate(req);
    if(op=="snapshot") return snapshot();
    if(op=="changes") return changes(req);
    if(op=="forget") return forget(req);
    if(op=="save") return save(req);
    if(op=="load") return load(req);
    if(op=="query") {
      auto out=envelope(); out["cells"]=json::array();
      require(req.at("points").is_array() && req.at("points").size()<=250000,"invalid_request","Invalid query points");
      for(const auto& p:req.at("points")) out["cells"].push_back(cell(key_for(vector3(p))));
      return out;
    }
    if(op=="status") { auto out=envelope(); out["leaf_count"]=tree->getNumLeafNodes(); out["memory_bytes"]=tree->memoryUsage(); out["stream_count"]=streams.size(); out["known_near_hit_cells"]=known_near_hits.size(); return out; }
    throw Error("unknown_operation","Unknown op: "+op);
  }
};

int main() {
  Service service;
  std::string line;
  while(std::getline(std::cin,line)) {
    json response,request;
    try {
      require(line.size()<=64*1024*1024,"request_too_large","JSONL request exceeds 64 MiB");
      request=json::parse(line); response=service.dispatch(request); response["ok"]=true;
    } catch(const Error& e) { response={{"ok",false},{"error",e.code},{"message",e.what()}};
    } catch(const std::exception& e) { response={{"ok",false},{"error","invalid_request"},{"message",e.what()}}; }
    if(request.is_object() && request.contains("request_id")) response["request_id"]=request["request_id"];
    std::cout<<response.dump()<<'\n'<<std::flush;
  }
}
