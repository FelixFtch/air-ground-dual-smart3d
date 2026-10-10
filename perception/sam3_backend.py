"""One offline SAM3 Video model, independently scoped camera sessions.

Adapted from the reviewed car1_sam3_video_people.py; no HTTP or simulator API.
Importing this module does not import torch, allocate CUDA, or load weights.
"""
from dataclasses import dataclass
import hashlib
import importlib
import json
import os
from pathlib import Path
import time
import uuid

RUNTIME = Path('/data/robotics/sam3_video_people')
IDENTITY = {'name': 'facebook/sam3', 'implementation': 'Transformers Sam3VideoModel',
            'revision': '3c879f39826c281e95690f02c7821c4de09afae7',
            'mode': 'streaming_video', 'query': 'person', 'dtype': 'bfloat16',
            'quantized': False, 'tracking_ids': 'native_sam3_session'}


def file_sha(path):
    digest = hashlib.sha256()
    with open(path, 'rb') as stream:
        for chunk in iter(lambda: stream.read(8 * 1024 * 1024), b''):
            digest.update(chunk)
    return digest.hexdigest()


@dataclass
class CameraSession:
    native: object
    session_id: str
    frame_index: int = 0
    reset_reason: str = 'service_started'


class VideoModel:
    def __init__(self, runtime=RUNTIME):
        runtime = Path(runtime)
        os.environ.update(HF_HUB_OFFLINE='1', TRANSFORMERS_OFFLINE='1',
                          HF_HOME=str(runtime / 'cache/huggingface'),
                          TOKENIZERS_PARALLELISM='false')
        from .vendor.sam3_video_memory import REVIEWED_SOURCES, configure_streaming_model
        # Refuse a changed upstream implementation before loading any weights.
        for name, digest in REVIEWED_SOURCES.items():
            module = importlib.import_module(name)
            if file_sha(module.__file__) != digest:
                raise ValueError('unreviewed_sam3_transformers_source')
        import torch
        from transformers import Sam3VideoModel, Sam3VideoProcessor
        from transformers.models.sam3_video import modeling_sam3_video
        from kernels import get_local_kernel
        self.torch = torch
        torch.set_num_threads(4)
        kernel_receipt = json.loads((runtime / 'receipts/kernels.json').read_text())
        if kernel_receipt.get('status') != 'complete' or kernel_receipt.get('torch') != torch.__version__:
            raise ValueError('sam3_postprocessing_kernels_not_validated')
        kernel_snapshot = (runtime / 'cache/huggingface/hub/kernels--kernels-community--cv-utils/snapshots'
                           / 'd473541808fc8b1595154771cc0fb18cef105caf')
        checked = 0
        for record in kernel_receipt['cache_files']:
            path = Path(record['path'])
            if '__pycache__' in path.parts:
                continue
            if (not path.is_relative_to(kernel_snapshot)
                    or not path.resolve().is_relative_to(kernel_snapshot.parents[1].resolve())
                    or file_sha(path) != record['sha256']):
                raise ValueError('sam3_postprocessing_kernel_hash_mismatch')
            checked += 1
        if not checked:
            raise ValueError('sam3_empty_kernel_receipt')
        modeling_sam3_video.cv_utils_kernel = get_local_kernel(kernel_snapshot)
        if not modeling_sam3_video.cv_utils_kernel:
            raise RuntimeError('sam3_offline_postprocessing_kernel_unavailable')
        model_path = runtime / 'model'
        receipt = json.loads((runtime / 'receipts/download.json').read_text())
        if (receipt.get('status') != 'complete' or receipt.get('model_id') != IDENTITY['name']
                or receipt.get('revision') != IDENTITY['revision']
                or model_path.resolve() != Path(receipt['path']).resolve()):
            raise ValueError('sam3_checkpoint_receipt_mismatch')
        records = receipt.get('files', [])
        if not any(r['name'] == 'model.safetensors' for r in records):
            raise ValueError('sam3_weight_receipt_missing')
        for record in records:
            path = model_path / record['name']
            if (Path(record['name']).name != record['name']
                    or not path.resolve().is_relative_to(model_path.resolve().parents[1])
                    or path.stat().st_size != record['bytes'] or file_sha(path) != record['sha256']):
                raise ValueError('sam3_checkpoint_hash_mismatch')
        self.processor = Sam3VideoProcessor.from_pretrained(model_path, local_files_only=True)
        self.model = Sam3VideoModel.from_pretrained(model_path, local_files_only=True,
            dtype=torch.bfloat16, attn_implementation='sdpa').to('cuda').eval()
        self.memory_policy = configure_streaming_model(self.model)
        self.sessions = {}

    def reset(self, source_key, reason='service_started'):
        from .vendor.sam3_video_memory import configure_streaming_session
        native = self.processor.init_video_session(
            inference_device='cuda', inference_state_device='cpu', processing_device='cpu',
            video_storage_device='cpu', dtype=self.torch.bfloat16, max_vision_features_cache_size=1)
        self.processor.add_text_prompt(inference_session=native, text='person')
        configure_streaming_session(native, self.model)
        self.sessions[source_key] = CameraSession(native, uuid.uuid4().hex, reset_reason=reason)
        return self.sessions[source_key]

    def drop(self, source_key):
        self.sessions.pop(source_key, None)

    def infer(self, source_key, image):
        """Called serially; model concurrency is deliberately unsupported."""
        from .vendor.sam3_video_memory import trim_streaming_session, expire_lost_streaming_tracks
        state = self.sessions.get(source_key) or self.reset(source_key)
        session, torch = state.native, self.torch
        with torch.inference_mode():
            torch.cuda.synchronize()
            started = time.monotonic()
            inputs = self.processor(images=image, device='cpu', return_tensors='pt')
            frame = inputs.pixel_values[0].to('cuda', dtype=torch.bfloat16)
            outputs = self.model(inference_session=session, frame=frame,
                                 frame_idx=state.frame_index, reverse=False)
            processed = self.processor.postprocess_outputs(session, outputs, original_sizes=inputs.original_sizes)
            masks = processed['masks'].detach().cpu().numpy()
            tracks = [dict(id=int(obj), box=[float(v) for v in box], score=float(score))
                      for obj, box, score in zip(processed['object_ids'].tolist(),
                          processed['boxes'].cpu().tolist(), processed['scores'].tolist())]
            nonempty = [i for i, mask in enumerate(masks) if mask.any()]
            tracks, masks = [tracks[i] for i in nonempty], masks[nonempty]
            lifecycle = expire_lost_streaming_tracks(session, state.frame_index,
                [track['id'] for track in tracks], time.monotonic(), self.model.config)
            memory = trim_streaming_session(session, state.frame_index, self.model.config)
            torch.cuda.synchronize()
            result = dict(tracks=tracks, masks=masks, inference_ms=(time.monotonic()-started)*1000,
                session_frame_index=state.frame_index, tracking_session_id=state.session_id,
                reset_reason=state.reset_reason, memory=memory, memory_policy=self.memory_policy, lifecycle=lifecycle)
            state.frame_index += 1
            return result
