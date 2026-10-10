// Copyright (c) 2026 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "StaticMeshFactory.h"
#include "UavKinematicActor.h"
#include "Carla/Actor/ActorBlueprintFunctionLibrary.h"
#include "Carla/Game/CarlaEpisode.h"

#include <util/ue-header-guard-begin.h>
#include "Engine/StaticMeshActor.h"
#include <util/ue-header-guard-end.h>

TArray<FActorDefinition> AStaticMeshFactory::GetDefinitions()
{
  using ABFL = UActorBlueprintFunctionLibrary;
  auto StaticMeshDefinition = ABFL::MakeGenericDefinition(
      TEXT("static"),
      TEXT("prop"),
      TEXT("mesh"));
  StaticMeshDefinition.Class = AStaticMeshActor::StaticClass();
  StaticMeshDefinition.Variations.Emplace(FActorVariation{
      TEXT("mesh_path"),
      EActorAttributeType::String,
      {""}, false});
  StaticMeshDefinition.Variations.Emplace(FActorVariation{
      TEXT("mass"),
      EActorAttributeType::Float,
      {""}, false});
  StaticMeshDefinition.Variations.Emplace(FActorVariation{
      TEXT("scale"),
      EActorAttributeType::Float,
      {"1.0f"}, false});
  // Keep the existing Go2 identity and one-piece kinematic mesh.
  auto Go2Definition = ABFL::MakeGenericDefinition(
      TEXT("static"), TEXT("prop"), TEXT("unitree_go2"));
  Go2Definition.Class = AStaticMeshActor::StaticClass();
  Go2Definition.Variations.Emplace(FActorVariation{
      TEXT("role_name"), EActorAttributeType::String,
      {TEXT("vision_nav_car1")}, false});
  // AirGroundLab only: a distinct XYZ body; never alter Go2 behavior.
  auto UavDefinition = ABFL::MakeGenericDefinition(
      TEXT("robot"), TEXT("uav"), TEXT("fighter_drone"));
  UavDefinition.Class = AUavKinematicActor::StaticClass();
  UavDefinition.Variations.Emplace(FActorVariation{
      TEXT("role_name"), EActorAttributeType::String, {TEXT("ag_uav_01")}, false});
  UavDefinition.Variations.Emplace(FActorVariation{TEXT("body_half_x_m"), EActorAttributeType::Float, {TEXT(".48")}, false});
  UavDefinition.Variations.Emplace(FActorVariation{TEXT("body_half_y_m"), EActorAttributeType::Float, {TEXT(".48")}, false});
  UavDefinition.Variations.Emplace(FActorVariation{TEXT("body_half_z_m"), EActorAttributeType::Float, {TEXT(".34")}, false});
  UavDefinition.Variations.Emplace(FActorVariation{TEXT("visual_scale"), EActorAttributeType::Float, {TEXT(".35")}, false});
  UavDefinition.Variations.Emplace(FActorVariation{TEXT("visual_yaw_degrees"), EActorAttributeType::Float, {TEXT("-90.0")}, false});
  UavDefinition.Variations.Emplace(FActorVariation{TEXT("max_speed_mps"), EActorAttributeType::Float, {TEXT("1.5")}, false});
  UavDefinition.Variations.Emplace(FActorVariation{TEXT("max_acceleration_mps2"), EActorAttributeType::Float, {TEXT("2.0")}, false});
  UavDefinition.Variations.Emplace(FActorVariation{TEXT("max_yaw_rate_rps"), EActorAttributeType::Float, {TEXT(".8")}, false});
  UavDefinition.Variations.Emplace(FActorVariation{TEXT("max_yaw_acceleration_rps2"), EActorAttributeType::Float, {TEXT("2.0")}, false});
  UavDefinition.Variations.Emplace(FActorVariation{TEXT("max_request_translation_m"), EActorAttributeType::Float, {TEXT(".15")}, false});
  UavDefinition.Variations.Emplace(FActorVariation{TEXT("flight_min_x_m"), EActorAttributeType::Float, {TEXT("-10.0")}, false});
  UavDefinition.Variations.Emplace(FActorVariation{TEXT("flight_min_y_m"), EActorAttributeType::Float, {TEXT("-10.0")}, false});
  UavDefinition.Variations.Emplace(FActorVariation{TEXT("flight_min_z_m"), EActorAttributeType::Float, {TEXT(".2")}, false});
  UavDefinition.Variations.Emplace(FActorVariation{TEXT("flight_max_x_m"), EActorAttributeType::Float, {TEXT("10.0")}, false});
  UavDefinition.Variations.Emplace(FActorVariation{TEXT("flight_max_y_m"), EActorAttributeType::Float, {TEXT("10.0")}, false});
  UavDefinition.Variations.Emplace(FActorVariation{TEXT("flight_max_z_m"), EActorAttributeType::Float, {TEXT("6.2")}, false});
  return { StaticMeshDefinition, Go2Definition, UavDefinition };
}

void AStaticMeshFactory::ReleaseContentPack(const FString &MountPoint)
{
  for (auto It = MeshCacheByPath.CreateIterator(); It; ++It)
  {
    if (It.Key().StartsWith(MountPoint))
    {
      It.RemoveCurrent();
    }
  }
}

FActorSpawnResult AStaticMeshFactory::SpawnActor(
    const FTransform &SpawnAtTransform,
    const FActorDescription &ActorDescription)
{
  using ABFL = UActorBlueprintFunctionLibrary;
  auto *World = GetWorld();
  if (World == nullptr)
  {
    UE_LOG(LogCarla, Error, TEXT
        ("AStaticMeshFactory: cannot spawn mesh into an empty world."));
    return {};
  }

  if (ActorDescription.Id == TEXT("robot.uav.fighter_drone"))
  {
    FActorSpawnParameters Parameters;
    Parameters.SpawnCollisionHandlingOverride = ESpawnActorCollisionHandlingMethod::AlwaysSpawn;
    auto *Uav = World->SpawnActor<AUavKinematicActor>(
        AUavKinematicActor::StaticClass(), SpawnAtTransform, Parameters);
    if (!Uav || !Uav->Configure(ActorDescription))
    {
      UE_LOG(LogCarla, Warning, TEXT("UAV spawn refused: %s"),
          Uav ? *Uav->GetLastMoveReason() : TEXT("spawn_failed"));
      if (Uav) World->DestroyActor(Uav);
      FActorSpawnResult Failed;
      Failed.Status = EActorSpawnResultStatus::InvalidDescription;
      return Failed;
    }
    return FActorSpawnResult(Uav);
  }

  const bool IsGo2 = ActorDescription.Id == TEXT("static.prop.unitree_go2");
  float Scale = ABFL::RetrieveActorAttributeToFloat("scale", ActorDescription.Variations, 1.0f);
  FTransform ScaledTransform(SpawnAtTransform);
  ScaledTransform.SetScale3D(FVector(Scale));

  // Resolve the mesh before spawning anything: an unloadable mesh_path must
  // fail the spawn instead of leaving a mesh-less actor (infinite bounds).
  FString MeshPath;
  UStaticMesh* Mesh = nullptr;
  if (IsGo2 || ActorDescription.Variations.Contains("mesh_path"))
  {
    MeshPath = IsGo2
        ? TEXT("/Game/Go2Official/SM_Go2_Stance.SM_Go2_Stance")
        : ABFL::ActorAttributeToString(ActorDescription.Variations["mesh_path"], "");
    if (MeshPath.IsEmpty())
    {
      // Same outcome as an unloadable path: nothing to render, no semantic tag,
      // infinite bounds. Refuse instead of spawning a mesh-less actor.
      UE_LOG(LogCarla, Error,
          TEXT("AStaticMeshFactory: empty mesh_path for actor %s; spawn refused."),
          *ActorDescription.Id);
      FActorSpawnResult Failed;
      Failed.Status = EActorSpawnResultStatus::InvalidDescription;
      return Failed;
    }
    else
    {
      if (TObjectPtr<UStaticMesh>* Cached = MeshCacheByPath.Find(MeshPath))
      {
        Mesh = Cached->Get();
      }
      else
      {
        Mesh = LoadObject<UStaticMesh>(nullptr, *MeshPath);
        if (Mesh != nullptr)
        {
          MeshCacheByPath.Add(MeshPath, Mesh);
        }
      }
      if (Mesh == nullptr)
      {
        UE_LOG(LogCarla, Error,
            TEXT("AStaticMeshFactory: failed to load mesh '%s' for actor %s; spawn refused."),
            *MeshPath, *ActorDescription.Id);
        FActorSpawnResult Failed;
        Failed.Status = EActorSpawnResultStatus::InvalidDescription;
        return Failed;
      }
    }
  }

  FActorSpawnParameters SpawnParameters;
  SpawnParameters.SpawnCollisionHandlingOverride =
      ESpawnActorCollisionHandlingMethod::AlwaysSpawn;
  auto *StaticMeshActor = World->SpawnActor<AStaticMeshActor>(
      ActorDescription.Class, ScaledTransform, SpawnParameters);

  auto *StaticMeshComponent = Cast<UStaticMeshComponent>(
      StaticMeshActor->GetRootComponent());
    
  if (StaticMeshComponent)
  {
    if (IsGo2)
    {
      StaticMeshComponent->SetMobility(EComponentMobility::Movable);
      if (!StaticMeshComponent->SetStaticMesh(Mesh))
      {
        World->DestroyActor(StaticMeshActor);
        return {};
      }
      StaticMeshActor->Tags.AddUnique(TEXT("Go2OfficialRigid"));
      StaticMeshComponent->SetCollisionProfileName(TEXT("BlockAllDynamic"));
      StaticMeshComponent->SetCollisionEnabled(ECollisionEnabled::QueryAndPhysics);
      StaticMeshComponent->SetSimulatePhysics(false);
      return FActorSpawnResult(StaticMeshActor);
    }
    if (ActorDescription.Variations.Contains("mesh_path"))
    {
      StaticMeshComponent->SetMobility(EComponentMobility::Movable);
      if (!StaticMeshComponent->SetStaticMesh(Mesh))
      {
        UE_LOG(LogCarla, Warning,
            TEXT("AStaticMeshFactory: failed to set mesh '%s' on actor %s."),
            *MeshPath, *ActorDescription.Id);
      }
      StaticMeshComponent->SetMobility(EComponentMobility::Static);

      if (ActorDescription.Variations.Contains("mass"))
      {
        float Mass = ABFL::RetrieveActorAttributeToFloat("mass", ActorDescription.Variations, 0.0f);
        if (Mass > 0)
        {
          StaticMeshComponent->SetMobility(EComponentMobility::Movable);
          StaticMeshComponent->SetSimulatePhysics(true);
          StaticMeshComponent->SetCollisionProfileName("PhysicsActor");
          StaticMeshComponent->SetMassOverrideInKg("", Mass);
        }
      }
    }
  }
  return FActorSpawnResult(StaticMeshActor);
}
