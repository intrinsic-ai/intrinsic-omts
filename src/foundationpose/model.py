# SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
# Copyright (c) 2024-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# SPDX-License-Identifier: Apache-2.0

import io
import os
import sys
import time

import triton_python_backend_utils as pb_utils

current_dir = os.path.dirname(os.path.abspath(__file__))
if current_dir not in sys.path:
  sys.path.insert(0, current_dir)

import cv2
import foundationpose_cpp
import numpy as np
import onnxruntime as ort
import trimesh


def fibonacci_sphere(samples=42):
  points = []
  phi = np.pi * (np.sqrt(5.0) - 1.0)  # golden angle in radians
  for i in range(samples):
    y = 1 - (i / float(samples - 1)) * 2
    radius = np.sqrt(1 - y * y)
    theta = phi * i
    x = np.cos(theta) * radius
    z = np.sin(theta) * radius
    points.append([x, y, z])
  return np.array(points, dtype=np.float32)


def look_at(eye, target, up=np.array([0, 1, 0], dtype=np.float32)):
  zaxis = eye - target
  zaxis = zaxis / (np.linalg.norm(zaxis) + 1e-8)
  xaxis = np.cross(up, zaxis)
  xaxis = xaxis / (np.linalg.norm(xaxis) + 1e-8)
  yaxis = np.cross(zaxis, xaxis)
  R = np.stack([xaxis, yaxis, zaxis], axis=1)
  return R


class TritonPythonModel:

  def initialize(self, args):
    sys.stderr.write(
        "[FoundationPose Triton Debug] Initializing TritonPythonModel...\n"
        f"  Python version: {sys.version}\n"
        f"  CUDA_LAUNCH_BLOCKING: {os.environ.get('CUDA_LAUNCH_BLOCKING')}\n"
        f"  ONNX Runtime version: {ort.__version__}\n"
        f"  ONNX Runtime available providers: {ort.get_available_providers()}\n"
    )
    sys.stderr.flush()

    self.glctx = foundationpose_cpp.RasterizeCudaContext(160, 160, 64)

    model_dir = os.path.dirname(os.path.abspath(__file__))
    refine_path = os.path.join(model_dir, "foundationpose_refine.onnx")
    score_path = os.path.join(model_dir, "foundationpose_score.onnx")

    providers = ["CUDAExecutionProvider", "CPUExecutionProvider"]
    sys.stderr.write(
        "[FoundationPose Triton] Loading ONNX Refine session from"
        f" {refine_path}...\n"
    )
    sys.stderr.flush()
    self.refine_session = ort.InferenceSession(refine_path, providers=providers)
    sys.stderr.write(
        "[FoundationPose Triton] Refine session providers:"
        f" {self.refine_session.get_providers()}\n"
    )
    sys.stderr.flush()

    sys.stderr.write(
        "[FoundationPose Triton] Loading ONNX Score session from"
        f" {score_path}...\n"
    )
    sys.stderr.flush()
    self.score_session = ort.InferenceSession(score_path, providers=providers)
    sys.stderr.write(
        "[FoundationPose Triton] Score session providers:"
        f" {self.score_session.get_providers()}\n"
    )
    sys.stderr.flush()

    sys.stderr.write(
        "[FoundationPose Triton] Initialized successfully with ONNX Runtime and"
        " native C++ CUDA Rasterizer.\n"
    )
    sys.stderr.flush()

  def _sample_initial_poses(self, mask, depth, K, num_views=40):
    mask_uint8 = np.ascontiguousarray((mask > 0).astype(np.uint8))
    depth_fp32 = np.ascontiguousarray(depth.astype(np.float32))
    K_fp32 = np.ascontiguousarray(K.astype(np.float32))
    poses = foundationpose_cpp.sample_initial_poses(
        mask_uint8, depth_fp32, K_fp32, num_views, 60.0
    )
    print(f"[FoundationPose Triton]: Sampled {len(poses)} candidate poses")
    return poses

  def _compute_crop_window_tf(
      self, poses_np, K, mesh_diameter, crop_ratio=1.2, out_size=(160, 160)
  ):
    return foundationpose_cpp.compute_crop_window_tf(
        np.ascontiguousarray(poses_np.astype(np.float32)),
        np.ascontiguousarray(K.astype(np.float32)),
        out_size[0],
        out_size[1],
        crop_ratio,
        mesh_diameter,
    )

  def _prepare_real_crop_6ch(
      self,
      rgb_np,
      depth_np,
      K_np,
      poses_np,
      mesh_diameter,
      res=(160, 160),
      xyz_map=None,
      rgb_float=None,
  ):
    N_cand = len(poses_np)
    H_orig, W_orig = depth_np.shape[:2]

    if rgb_float is None:
      if rgb_np.shape[:2] != (H_orig, W_orig):
        rgb_np = cv2.resize(rgb_np, (W_orig, H_orig))
      rgb_float = (
          (rgb_np.astype(np.float32) / 255.0)
          if rgb_np.dtype == np.uint8
          else rgb_np.astype(np.float32)
      )

    if xyz_map is None:
      fx, fy, cx, cy = K_np[0, 0], K_np[1, 1], K_np[0, 2], K_np[1, 2]
      ys_grid, xs_grid = np.indices((H_orig, W_orig), dtype=np.float32)
      X_cam = (xs_grid - cx) * depth_np / fx
      Y_cam = (ys_grid - cy) * depth_np / fy
      Z_cam = depth_np.copy()
      xyz_map = np.stack([X_cam, Y_cam, Z_cam], axis=-1).astype(np.float32)

    tfs = self._compute_crop_window_tf(
        poses_np, K_np, mesh_diameter=mesh_diameter, out_size=res
    )

    batch_patches = np.zeros((N_cand, res[0], res[1], 6), dtype=np.float32)
    radius_norm = mesh_diameter / 2.0

    for i in range(N_cand):
      M_tf = tfs[i]
      T_i = poses_np[i, :3, 3]

      crop_rgb = cv2.warpPerspective(
          rgb_float, M_tf, res, flags=cv2.INTER_LINEAR
      )
      crop_xyz = cv2.warpPerspective(
          xyz_map, M_tf, res, flags=cv2.INTER_NEAREST
      )

      valid_mask = (crop_xyz[:, :, 2] > 0.05).astype(np.float32)
      crop_xyz_norm = np.zeros_like(crop_xyz)
      crop_xyz_norm[:, :, 0] = (crop_xyz[:, :, 0] - T_i[0]) / radius_norm
      crop_xyz_norm[:, :, 1] = (crop_xyz[:, :, 1] - T_i[1]) / radius_norm
      crop_xyz_norm[:, :, 2] = (crop_xyz[:, :, 2] - T_i[2]) / radius_norm
      crop_xyz_norm = crop_xyz_norm * valid_mask[:, :, None]

      batch_patches[i, :, :, :3] = crop_rgb
      batch_patches[i, :, :, 3:] = crop_xyz_norm

    batch_patches = np.ascontiguousarray(batch_patches, dtype=np.float32)
    tfs = np.ascontiguousarray(tfs, dtype=np.float32)
    return batch_patches, tfs

  def _render_views_nvdiffrast_6ch(
      self,
      v_np,
      f_np,
      candidate_poses,
      K_np,
      tfs_np,
      mesh_diameter,
      res=(160, 160),
  ):
    N_cand = len(candidate_poses)
    R_cand = candidate_poses[:, :3, :3]
    T_cand = candidate_poses[:, :3, 3]  # (N_cand, 3)

    v_cam = (
        np.matmul(v_np[None, :, :], R_cand.transpose(0, 2, 1))
        + T_cand[:, None, :]
    )

    fx, fy, cx, cy = K_np[0, 0], K_np[1, 1], K_np[0, 2], K_np[1, 2]
    px = (v_cam[:, :, 0] * fx / v_cam[:, :, 2]) + cx
    py = (v_cam[:, :, 1] * fy / v_cam[:, :, 2]) + cy

    m00 = tfs_np[:, 0, 0][:, None]
    m02 = tfs_np[:, 0, 2][:, None]
    m11 = tfs_np[:, 1, 1][:, None]
    m12 = tfs_np[:, 1, 2][:, None]

    px_crop = m00 * px + m02
    py_crop = m11 * py + m12

    x_clip = 2.0 * px_crop / float(res[1]) - 1.0
    y_clip = 1.0 - 2.0 * py_crop / float(res[0])
    z_clip = (v_cam[:, :, 2] - 0.1) / (100.0 - 0.1)

    v_clip = np.ascontiguousarray(
        np.stack([x_clip, y_clip, z_clip, np.ones_like(z_clip)], axis=-1),
        dtype=np.float32,
    )
    f_np_contig = np.ascontiguousarray(f_np, dtype=np.int32)
    v_cam_contig = np.ascontiguousarray(v_cam, dtype=np.float32)

    rast = self.glctx.rasterize(v_clip, f_np_contig, res[0], res[1])
    xyz_interp = self.glctx.interpolate(
        v_cam_contig, rast, f_np_contig, res[0], res[1]
    )

    mask_rendered = (rast[:, :, :, 3:4] > 0).astype(np.float32)
    radius_norm = mesh_diameter / 2.0

    rendered_xyz_norm = (
        (xyz_interp - T_cand[:, None, None, :]) / radius_norm * mask_rendered
    )

    rendered_6ch = np.zeros((N_cand, res[0], res[1], 6), dtype=np.float32)
    rendered_6ch[:, :, :, :3] = 0.5 * mask_rendered
    rendered_6ch[:, :, :, 3:] = rendered_xyz_norm
    rendered_6ch = np.flip(rendered_6ch, axis=1)
    rendered_6ch = np.ascontiguousarray(rendered_6ch, dtype=np.float32)
    return rendered_6ch

  def _load_obj_mesh(self, cad_bytes):
    verts = []
    faces = []
    try:
      cad_str = (
          cad_bytes.decode("utf-8", errors="ignore")
          if isinstance(cad_bytes, bytes)
          else str(cad_bytes)
      )
      for line in cad_str.splitlines():
        line = line.strip()
        if not line or line.startswith("#"):
          continue
        parts = line.split()
        if not parts:
          continue
        if parts[0] == "v" and len(parts) >= 4:
          verts.append([float(parts[1]), float(parts[2]), float(parts[3])])
        elif parts[0] == "f" and len(parts) >= 4:
          face_verts = [int(p.split("/")[0]) - 1 for p in parts[1:]]
          if len(face_verts) == 3:
            faces.append(face_verts)
          elif len(face_verts) == 4:
            faces.append([face_verts[0], face_verts[1], face_verts[2]])
            faces.append([face_verts[0], face_verts[2], face_verts[3]])
    except Exception as e:
      sys.stderr.write(f"[Debug _load_obj_mesh error]: {e}\n")

    if len(verts) == 0:
      raise ValueError(
          "Failed to parse CAD OBJ mesh from CAD_MODEL_BYTES input tensor: no"
          " valid vertices found."
      )

    vertices = np.array(verts, dtype=np.float32)
    faces = np.array(faces, dtype=np.int32)
    sys.stderr.write(
        f"[Debug _load_obj_mesh parsed]: vertices={vertices.shape},"
        f" faces={faces.shape}\n"
    )
    sys.stderr.flush()
    return trimesh.Trimesh(vertices=vertices, faces=faces)

  def _load_glb_mesh(self, cad_bytes: bytes) -> trimesh.Trimesh:
    """Decodes raw .glb binary bytes into a single trimesh.Trimesh object."""
    try:
      # 1. Load the GLB from an in-memory byte stream
      loaded = trimesh.load(
          io.BytesIO(cad_bytes), file_type="glb", force="mesh"
      )
      # 2. GLB files can sometimes load as a trimesh.Scene; concatenate all geometries into one Trimesh
      if isinstance(loaded, trimesh.Scene):
        mesh = loaded.dump(concatenate=True)
      else:
        mesh = loaded
      sys.stderr.write(
          f"[Debug _load_glb_mesh parsed]: vertices={mesh.vertices.shape},"
          f" faces={mesh.faces.shape}\n"
      )
      sys.stderr.flush()
      return mesh
    except Exception as e:
      sys.stderr.write(
          f"[Debug _load_glb_mesh failed, falling back to OBJ]: {e}\n"
      )
      sys.stderr.flush()
      return self._load_obj_mesh(cad_bytes)

  def execute(self, requests):
    responses = []
    for req_idx, request in enumerate(requests):
      try:
        t0 = time.perf_counter()
        rgb_np = pb_utils.get_input_tensor_by_name(request, "RGB").as_numpy()
        depth_np = pb_utils.get_input_tensor_by_name(
            request, "DEPTH"
        ).as_numpy()
        mask_np = pb_utils.get_input_tensor_by_name(request, "MASK").as_numpy()
        K_np = pb_utils.get_input_tensor_by_name(request, "CAM_K").as_numpy()

        if rgb_np.ndim == 4:
          rgb_np = rgb_np[0]
        if depth_np.ndim == 3:
          depth_np = depth_np[0]
        if K_np.ndim == 3:
          K_np = K_np[0]
        if mask_np.ndim == 2:
          mask_np = np.expand_dims(mask_np, axis=0)
        elif mask_np.ndim == 4:
          mask_np = mask_np[0]

        cad_input = pb_utils.get_input_tensor_by_name(
            request, "CAD_MODEL_BYTES"
        ).as_numpy()
        cad_bytes = cad_input.astype(np.uint8).tobytes()

        sys.stderr.write(
            f"[FoundationPose Execute Debug] Request {req_idx}:"
            f" RGB={rgb_np.shape} ({rgb_np.dtype}), DEPTH={depth_np.shape}"
            f" ({depth_np.dtype}), MASK={mask_np.shape} ({mask_np.dtype}),"
            f" K={K_np.shape} ({K_np.dtype}), CAD_BYTES={len(cad_bytes)}"
            " bytes\n"
        )
        sys.stderr.flush()

        mesh = self._load_glb_mesh(cad_bytes)
        mesh_vertices = np.ascontiguousarray(mesh.vertices, dtype=np.float32)
        mesh_faces = np.ascontiguousarray(mesh.faces, dtype=np.int32)
        if len(mesh_vertices) > 0:
          bbox_min = mesh_vertices.min(axis=0)
          bbox_max = mesh_vertices.max(axis=0)
          mesh_diameter = float(np.linalg.norm(bbox_max - bbox_min))
        else:
          mesh_diameter = 0.05
        mesh_diameter = max(mesh_diameter, 0.05)

        sys.stderr.write(
            "[FoundationPose Execute Debug] Mesh parsed:"
            f" vertices={mesh_vertices.shape}"
            f" (contiguous={mesh_vertices.flags['C_CONTIGUOUS']}),"
            f" faces={mesh_faces.shape}"
            f" (contiguous={mesh_faces.flags['C_CONTIGUOUS']}),"
            f" diameter={mesh_diameter:.4f}\n"
        )
        sys.stderr.flush()

        num_iterations = 5
        num_iter_tensor = pb_utils.get_input_tensor_by_name(
            request, "NUM_ITERATIONS"
        )
        if num_iter_tensor is not None:
          try:
            num_iterations = int(num_iter_tensor.as_numpy().flatten()[0])
          except Exception as e:
            sys.stderr.write(
                "[WARNING] Could not parse NUM_ITERATIONS tensor, defaulting to"
                f" {num_iterations}: {e}\n"
            )

        batch_size = 240
        batch_size_tensor = pb_utils.get_input_tensor_by_name(
            request, "BATCH_SIZE"
        )
        if batch_size_tensor is not None:
          try:
            batch_size = int(batch_size_tensor.as_numpy().flatten()[0])
          except Exception as e:
            sys.stderr.write(
                "[WARNING] Could not parse BATCH_SIZE tensor, defaulting to"
                f" {batch_size}: {e}\n"
            )

        # Precompute normalized RGB and camera point map once per request
        H_orig, W_orig = depth_np.shape[:2]
        if rgb_np.shape[:2] != (H_orig, W_orig):
          rgb_np = cv2.resize(rgb_np, (W_orig, H_orig))
        rgb_float = (
            (rgb_np.astype(np.float32) / 255.0)
            if rgb_np.dtype == np.uint8
            else rgb_np.astype(np.float32)
        )
        fx, fy, cx, cy = K_np[0, 0], K_np[1, 1], K_np[0, 2], K_np[1, 2]
        ys_grid, xs_grid = np.indices((H_orig, W_orig), dtype=np.float32)
        X_cam = (xs_grid - cx) * depth_np / fx
        Y_cam = (ys_grid - cy) * depth_np / fy
        Z_cam = depth_np.copy()
        xyz_map = np.stack([X_cam, Y_cam, Z_cam], axis=-1).astype(np.float32)

        rotations = []
        translations = []
        confidences = []

        for mask_idx in range(len(mask_np)):
          sys.stderr.write(
              "[FoundationPose Execute Debug] Processing mask"
              f" {mask_idx+1}/{len(mask_np)}...\n"
          )
          sys.stderr.flush()
          single_mask_np = mask_np[mask_idx]
          num_cand = 40
          candidate_poses_np = self._sample_initial_poses(
              single_mask_np, depth_np, K_np, num_views=num_cand
          )
          total_cands = len(candidate_poses_np)
          effective_batch_size = (
              total_cands
              if (batch_size <= 0 or batch_size >= total_cands)
              else batch_size
          )

          sys.stderr.write(
              "[FoundationPose Execute Debug] Candidate poses:"
              f" total={total_cands}, batch_size={effective_batch_size}\n"
          )
          sys.stderr.flush()

          best_score = -float("inf")
          raw_best_pose = None

          for chunk_start in range(0, total_cands, effective_batch_size):
            chunk_end = min(chunk_start + effective_batch_size, total_cands)
            chunk_poses_np = candidate_poses_np[chunk_start:chunk_end].copy()

            for iteration in range(num_iterations):
              in2, tfs_np = self._prepare_real_crop_6ch(
                  rgb_np,
                  depth_np,
                  K_np,
                  chunk_poses_np,
                  mesh_diameter,
                  xyz_map=xyz_map,
                  rgb_float=rgb_float,
              )
              in1 = self._render_views_nvdiffrast_6ch(
                  mesh_vertices,
                  mesh_faces,
                  chunk_poses_np,
                  K_np,
                  tfs_np,
                  mesh_diameter,
              )

              sys.stderr.write(
                  f"  [Refine Iteration {iteration+1}/{num_iterations}]"
                  f" chunk [{chunk_start}:{chunk_end}]"
                  f" in1={in1.shape} ({in1.dtype},"
                  f" contiguous={in1.flags['C_CONTIGUOUS']}), in2={in2.shape}"
                  f" ({in2.dtype}, contiguous={in2.flags['C_CONTIGUOUS']})...\n"
              )
              sys.stderr.flush()

              refine_outs = self.refine_session.run(
                  None, {"input1": in1, "input2": in2}
              )
              trans_delta_np = refine_outs[0].astype(np.float32)
              rot_delta_np = refine_outs[1].astype(np.float32)

              chunk_poses_np = foundationpose_cpp.update_refined_poses(
                  chunk_poses_np,
                  trans_delta_np,
                  rot_delta_np,
                  mesh_diameter,
                  0.34906585,
              )

            in2, tfs_np = self._prepare_real_crop_6ch(
                rgb_np,
                depth_np,
                K_np,
                chunk_poses_np,
                mesh_diameter,
                xyz_map=xyz_map,
                rgb_float=rgb_float,
            )
            in1 = self._render_views_nvdiffrast_6ch(
                mesh_vertices,
                mesh_faces,
                chunk_poses_np,
                K_np,
                tfs_np,
                mesh_diameter,
            )

            sys.stderr.write(
                f"  [Score Run] chunk [{chunk_start}:{chunk_end}]"
                f" in1={in1.shape} ({in1.dtype},"
                f" contiguous={in1.flags['C_CONTIGUOUS']}), in2={in2.shape}"
                f" ({in2.dtype}, contiguous={in2.flags['C_CONTIGUOUS']})...\n"
            )
            sys.stderr.flush()

            score_outs = self.score_session.run(
                None, {"input1": in1, "input2": in2}
            )
            scores = score_outs[0].reshape(-1)

            chunk_best_local_idx = int(np.argmax(scores))
            chunk_best_score = float(scores[chunk_best_local_idx])
            if chunk_best_score > best_score:
              best_score = chunk_best_score
              raw_best_pose = chunk_poses_np[chunk_best_local_idx]

          if raw_best_pose is None:
            raw_best_pose = np.eye(4, dtype=np.float32)
            best_score = 0.0

          mesh_center = (
              (mesh_vertices.min(axis=0) + mesh_vertices.max(axis=0)) / 2.0
          ).astype(np.float32)
          final_best_pose = foundationpose_cpp.apply_mesh_center_offset(
              raw_best_pose, mesh_center
          )

          best_R_arr = np.array(
              final_best_pose[:3, :3], dtype=np.float32, copy=True
          )
          best_T_arr = np.array(
              final_best_pose[:3, 3], dtype=np.float32, copy=True
          )
          best_S_arr = np.array(
              [float(best_score)], dtype=np.float32, copy=True
          )
          rotations.append(best_R_arr)
          translations.append(best_T_arr)
          confidences.append(best_S_arr)

        if len(mask_np) == 0:
          out_r_arr = np.zeros((0, 3, 3), dtype=np.float32)
          out_t_arr = np.zeros((0, 3), dtype=np.float32)
          out_s_arr = np.zeros((0, 1), dtype=np.float32)
        else:
          out_r_arr = np.stack(rotations, axis=0).astype(np.float32)
          out_t_arr = np.stack(translations, axis=0).astype(np.float32)
          out_s_arr = np.stack(confidences, axis=0).astype(np.float32)

        out_r = pb_utils.Tensor("ROTATION", out_r_arr)
        out_t = pb_utils.Tensor("TRANSLATION", out_t_arr)
        out_s = pb_utils.Tensor("CONFIDENCE", out_s_arr)

        sys.stderr.write(
            f"[FoundationPose Execute Debug] Request {req_idx} completed in"
            f" {(time.perf_counter() - t0):.3f}s with {len(rotations)}"
            " outputs.\n"
        )
        sys.stderr.flush()

        responses.append(
            pb_utils.InferenceResponse(output_tensors=[out_r, out_t, out_s])
        )
      except Exception as e:
        import traceback

        sys.stderr.write(
            "[ERROR in FoundationPose execute]:"
            f" {e}\n{traceback.format_exc()}\n"
        )
        sys.stderr.flush()
        responses.append(
            pb_utils.InferenceResponse(
                output_tensors=[], error=pb_utils.TritonError(str(e))
            )
        )

    return responses
