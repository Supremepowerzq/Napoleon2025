# -*- coding: utf-8 -*-
"""
BronchusDataCollector — 增强版专家演示录制器
==============================================
在专家手动操作期间同步录制：
  · 电机角度序列（M0/M1/M2）
  · 每帧结构化视觉特征（结石/岔口/深度）
  · 控制模式标签（导航/清理）
  · 路径标签（通向哪条主支气管）

存储格式：HDF5（支持高效随机访问和压缩）
调用方式：在主控程序的控制循环中每帧调用 collect_frame()

Author: ZQ  Date: 2026-05
"""

import os
import time
import threading
from datetime import datetime
from dataclasses import dataclass, asdict, field
from typing import Optional, Dict, List, Any

import numpy as np

try:
    import h5py
    HDF5_AVAILABLE = True
except ImportError:
    HDF5_AVAILABLE = False
    print("[DataCollector] 警告: h5py 未安装，将使用 npz 格式。运行: pip install h5py")

from model import (
    OBS_FIELDS, BRONCHUS_PATHS, NUM_GOALS,
    YOLO_CODE_TO_IDX, IDX_TO_YOLO, IDX_TO_NAME,
    path_idx,
    TASK_NAVIGATE, TASK_CLEAR,
    build_obs_vector,
)


# ──────────────────────────────────────────────────────────────
# 全局共享特征字典（视觉线程写入，数据采集器读取）
# 在主程序 video_processing 线程中调用 update_visual_features() 更新
# ──────────────────────────────────────────────────────────────

_visual_features: Dict[str, Any] = {
    "stone_cx":         0.0,
    "stone_cy":         0.0,
    "stone_area":       0.0,
    "stone_detected":   False,
    "bifur_cx":         0.0,
    "bifur_cy":         0.0,
    "bifur_area":       0.0,
    "depth_center_mm":  0.0,
    "depth_mean_mm":    0.0,
    "depth_max_mm":     0.0,
    "path_left_mm":     0.0,
    "path_center_mm":   0.0,
    "path_right_mm":    0.0,
}
_vis_lock = threading.Lock()


def update_visual_features(**kwargs):
    """
    视觉处理线程调用此函数更新当前帧的视觉特征。
    在 predict_wqx-Bronchus.py 的主循环末尾调用。

    示例（在 predict_wqx-Bronchus.py 中添加）:
        from BC.data_collector import update_visual_features
        update_visual_features(
            stone_cx=cx_norm, stone_cy=cy_norm,
            stone_area=area_ratio, stone_detected=has_stone,
            bifur_cx=bfx, bifur_cy=bfy, bifur_area=bf_area,
            depth_center_mm=z_center, depth_mean_mm=z_mean, depth_max_mm=z_max,
            path_left_mm=path_l, path_center_mm=path_c, path_right_mm=path_r,
        )
    """
    with _vis_lock:
        _visual_features.update(kwargs)


def get_visual_features() -> Dict[str, Any]:
    """读取当前帧视觉特征（线程安全）。"""
    with _vis_lock:
        return dict(_visual_features)


# ──────────────────────────────────────────────────────────────
# 单帧数据结构
# ──────────────────────────────────────────────────────────────

@dataclass
class Frame:
    timestamp:      float       # 相对于episode开始的时间 (s)
    dt:             float = 0.05  # 与上一帧的时间间隔 (s)，20Hz=0.05
    # 电机位置
    m0_angle:       float = 0.0  # 度
    m1_angle:       float = 0.0
    m2_angle:       float = 0.0
    # 电机速度（°/s，由相邻帧 Δangle/Δt 计算）
    m0_vel:         float = 0.0
    m1_vel:         float = 0.0
    m2_vel:         float = 0.0
    # 专家动作（上一帧到本帧的角度变化量 °）
    delta_m0:       float = 0.0
    delta_m1:       float = 0.0
    delta_m2:       float = 0.0
    # 视觉特征（已归一化）
    stone_cx:       float = 0.0
    stone_cy:       float = 0.0
    stone_area:     float = 0.0
    stone_detected: float = 0.0
    bifur_cx:       float = 0.0
    bifur_cy:       float = 0.0
    bifur_area:     float = 0.0
    depth_center_mm: float = 0.0
    depth_mean_mm:  float = 0.0
    depth_max_mm:   float = 0.0
    path_left_mm:   float = 0.0
    path_center_mm: float = 0.0
    path_right_mm:  float = 0.0
    # 控制标签
    task_label:     int   = TASK_NAVIGATE   # 0=导航 1=清理


# ──────────────────────────────────────────────────────────────
# 录制器主类
# ──────────────────────────────────────────────────────────────

class BronchusDataCollector:
    """
    支气管专家演示数据采集器。

    使用流程
    --------
    1. 创建实例: collector = BronchusDataCollector(motor_group, path_label=2)
    2. 开始采集: collector.start("左主支气管_第1次")
    3. 在控制循环中调用: collector.collect_frame()  （每帧）
    4. 结束采集: filepath = collector.stop()
    5. 查看统计: collector.print_stats()
    """

    def __init__(
        self,
        motor_group,
        save_dir:    str   = "expert_demos",
        record_hz:   float = 20.0,
        path_label:  int   = 0,
        use_hdf5:    bool  = False,   # False=JSON（默认，可直接用 VS Code 打开）
    ):
        """
        Parameters
        ----------
        motor_group  : MotorGroup2025 实例（从主程序传入）
        save_dir     : 保存目录
        record_hz    : 目标采集频率（Hz），不超过主循环频率
        path_label   : 目标支气管标签（见 BRONCHUS_PATHS）
        use_hdf5     : 是否使用 HDF5 格式（默认 False，即 JSON）
                       JSON  → 可在 VS Code / 记事本直接查看，文件约 1-3 MB/分钟
                       HDF5  → 二进制压缩格式，需 h5py，文件约 0.1-0.3 MB/分钟
        """
        self.motor_group  = motor_group
        self.save_dir     = save_dir
        self.record_hz    = record_hz
        self.path_label   = path_label
        self.use_hdf5     = use_hdf5 and HDF5_AVAILABLE

        self._frames:     List[Frame] = []
        self._start_time: Optional[float] = None
        self._prev_angles: Optional[tuple] = None
        self._prev_time:   Optional[float] = None
        self._is_recording = False
        self._session_name = ""

        # 自动切换子任务（根据视觉特征判断）
        self.auto_task_label = True

        os.makedirs(save_dir, exist_ok=True)

    # ── 控制接口 ──────────────────────────────────────────────

    def start(self, session_name: str = "", path_label: Optional[int] = None) -> bool:
        """开始录制。返回 True 表示成功。"""
        if self._is_recording:
            print("[DataCollector] 已在录制中，请先停止")
            return False

        if path_label is not None:
            self.path_label = path_label

        self._frames = []
        self._start_time = time.time()
        self._prev_angles = None
        self._is_recording = True
        self._session_name = session_name or datetime.now().strftime("%Y%m%d_%H%M%S")

        yolo_code = IDX_TO_YOLO.get(self.path_label, "EXP")
        path_name = IDX_TO_NAME.get(self.path_label, "unknown")
        print(f"[DataCollector] 开始录制: {self._session_name}")
        print(f"  目标路径: [{yolo_code}] {path_name}")
        print(f"  采集频率: {self.record_hz} Hz  存储目录: {self.save_dir}")
        self._prev_time = None   # 用于计算 dt
        return True

    def collect_frame(self) -> bool:
        """
        在控制循环中每帧调用，录制当前帧数据。
        返回 True 表示录制成功，False 表示未在录制。
        """
        if not self._is_recording:
            return False

        now = time.time()
        ts  = now - self._start_time

        # 计算帧间隔 dt
        if self._prev_time is not None:
            dt = float(now - self._prev_time)
            dt = max(0.001, min(dt, 1.0))  # 限制在合理范围
        else:
            dt = 1.0 / self.record_hz      # 第一帧用额定步长
        self._prev_time = now

        # 读取电机角度
        try:
            if hasattr(self.motor_group, "get_cached_angles"):
                angles = self.motor_group.get_cached_angles()
            else:
                angles = self.motor_group.get_angles()
            m0, m1, m2 = float(angles[0]), float(angles[1]), float(angles[2])
        except Exception as e:
            print(f"[DataCollector] 读取电机角度失败: {e}")
            return False

        # 计算动作增量和电机速度
        if self._prev_angles is not None:
            dm0 = m0 - self._prev_angles[0]
            dm1 = m1 - self._prev_angles[1]
            dm2 = m2 - self._prev_angles[2]
            # 速度 = 角度差 / 时间差（°/s）
            v0 = dm0 / dt
            v1 = dm1 / dt
            v2 = dm2 / dt
        else:
            dm0 = dm1 = dm2 = 0.0
            v0  = v1  = v2  = 0.0
        self._prev_angles = (m0, m1, m2)

        # 跳过微小变化的帧（减少冗余），但每0.5s强制保留一帧
        if (self._frames and
                abs(dm0) < 0.05 and abs(dm1) < 0.05 and abs(dm2) < 0.05):
            last_ts = self._frames[-1].timestamp
            if ts - last_ts < 0.5:
                return True

        # 读取视觉特征
        vis = get_visual_features()

        # 自动判断子任务标签
        if self.auto_task_label:
            task = TASK_CLEAR if vis.get("stone_detected", False) else TASK_NAVIGATE
        else:
            task = TASK_NAVIGATE

        frame = Frame(
            timestamp=round(ts, 4),
            dt=round(dt, 4),
            m0_angle=m0, m1_angle=m1, m2_angle=m2,
            m0_vel=round(v0, 2), m1_vel=round(v1, 2), m2_vel=round(v2, 2),
            delta_m0=round(dm0, 3), delta_m1=round(dm1, 3), delta_m2=round(dm2, 3),
            stone_cx=vis.get("stone_cx", 0.0),
            stone_cy=vis.get("stone_cy", 0.0),
            stone_area=vis.get("stone_area", 0.0),
            stone_detected=float(vis.get("stone_detected", False)),
            bifur_cx=vis.get("bifur_cx", 0.0),
            bifur_cy=vis.get("bifur_cy", 0.0),
            bifur_area=vis.get("bifur_area", 0.0),
            depth_center_mm=vis.get("depth_center_mm", 0.0),
            depth_mean_mm=vis.get("depth_mean_mm", 0.0),
            depth_max_mm=vis.get("depth_max_mm", 0.0),
            path_left_mm=vis.get("path_left_mm", 0.0),
            path_center_mm=vis.get("path_center_mm", 0.0),
            path_right_mm=vis.get("path_right_mm", 0.0),
            task_label=task,
        )
        self._frames.append(frame)
        return True

    def stop(self) -> Optional[str]:
        """停止录制并保存文件。返回保存路径。"""
        if not self._is_recording:
            return None
        self._is_recording = False

        n = len(self._frames)
        if n < 10:
            print(f"[DataCollector] 录制帧数太少 ({n} 帧)，已丢弃")
            return None

        duration  = self._frames[-1].timestamp if self._frames else 0.0
        yolo_code = IDX_TO_YOLO.get(self.path_label, "EXP")
        path_name = IDX_TO_NAME.get(self.path_label, "unknown")

        # 文件名格式：<会话名>_<YOLO代码>_<帧数>frames.json / .h5
        ext   = ".h5" if self.use_hdf5 else ".json"
        fname = f"{self._session_name}_{yolo_code}_{n}frames{ext}"
        fpath = os.path.join(self.save_dir, fname)

        if self.use_hdf5:
            self._save_hdf5(fpath)
        else:
            self._save_json(fpath)

        print(f"[DataCollector] 保存完成: {fpath}")
        print(f"  {n} 帧  时长 {duration:.1f}s  路径: [{yolo_code}] {path_name}")
        self._frames = []
        return fpath

    # ── 存储 ──────────────────────────────────────────────────

    def _save_json(self, fpath: str):
        """
        保存为 JSON 格式，与 ActionRecorder 兼容，可直接用 VS Code 打开查看。

        格式示例：
        {
          "version": "2.0",
          "session_name": "20260530_201339",
          "yolo_code": "RMB",
          "path_name": "右主支气管",
          "path_label": 3,
          "n_frames": 101,
          "duration_s": 5.05,
          "record_hz": 20.0,
          "obs_fields": ["stone_cx", ...],
          "frames": [
            {"timestamp": 0.0, "dt": 0.05,
             "m0_angle": -719.0, "m1_angle": -6.0, "m2_angle": 360.0,
             "m0_vel": 0.0, "m1_vel": 0.0, "m2_vel": 0.0,
             "delta_m0": 0.0, "delta_m1": 0.0, "delta_m2": 0.0,
             "stone_cx": 0.0, ... "task_label": 0},
            ...
          ]
        }
        """
        import json as _json
        obs_names = [name for name, _, _ in OBS_FIELDS]
        data = {
            "version":      "2.0",
            "session_name": self._session_name,
            "yolo_code":    IDX_TO_YOLO.get(self.path_label, "EXP"),
            "path_name":    IDX_TO_NAME.get(self.path_label, "unknown"),
            "path_label":   self.path_label,
            "n_frames":     len(self._frames),
            "duration_s":   round(self._frames[-1].timestamp, 4) if self._frames else 0.0,
            "record_hz":    self.record_hz,
            "obs_fields":   obs_names,
            "frames": [
                {
                    "timestamp":    round(f.timestamp, 4),
                    "dt":           round(f.dt, 4),
                    "m0_angle":     round(f.m0_angle, 3),
                    "m1_angle":     round(f.m1_angle, 3),
                    "m2_angle":     round(f.m2_angle, 3),
                    "m0_vel":       round(f.m0_vel, 2),
                    "m1_vel":       round(f.m1_vel, 2),
                    "m2_vel":       round(f.m2_vel, 2),
                    "delta_m0":     round(f.delta_m0, 3),
                    "delta_m1":     round(f.delta_m1, 3),
                    "delta_m2":     round(f.delta_m2, 3),
                    "stone_cx":     round(f.stone_cx, 4),
                    "stone_cy":     round(f.stone_cy, 4),
                    "stone_area":   round(f.stone_area, 4),
                    "stone_detected": bool(f.stone_detected),
                    "bifur_cx":     round(f.bifur_cx, 4),
                    "bifur_cy":     round(f.bifur_cy, 4),
                    "bifur_area":   round(f.bifur_area, 4),
                    "depth_center_mm": round(f.depth_center_mm, 2),
                    "depth_mean_mm":   round(f.depth_mean_mm, 2),
                    "depth_max_mm":    round(f.depth_max_mm, 2),
                    "path_left_mm":    round(f.path_left_mm, 2),
                    "path_center_mm":  round(f.path_center_mm, 2),
                    "path_right_mm":   round(f.path_right_mm, 2),
                    "task_label":      f.task_label,
                }
                for f in self._frames
            ],
        }
        with open(fpath, "w", encoding="utf-8") as fp:
            _json.dump(data, fp, indent=2, ensure_ascii=False)

    def _save_hdf5(self, fpath: str):
        n = len(self._frames)
        # 构建 numpy 数组
        timestamps   = np.array([f.timestamp for f in self._frames], dtype=np.float32)
        motor_angles = np.array([[f.m0_angle, f.m1_angle, f.m2_angle] for f in self._frames], dtype=np.float32)
        actions      = np.array([[f.delta_m0, f.delta_m1, f.delta_m2] for f in self._frames], dtype=np.float32)
        task_labels  = np.array([f.task_label for f in self._frames], dtype=np.int8)

        # 观测矩阵（按 OBS_FIELDS 顺序）
        obs_names = [name for name, _, _ in OBS_FIELDS]
        obs = np.zeros((n, len(obs_names)), dtype=np.float32)
        for i, f in enumerate(self._frames):
            fd = asdict(f)
            for j, name in enumerate(obs_names):
                obs[i, j] = fd.get(name, 0.0)

        with h5py.File(fpath, "w") as hf:
            hf.attrs["session_name"]   = self._session_name
            hf.attrs["path_label"]     = self.path_label
            hf.attrs["yolo_code"]      = IDX_TO_YOLO.get(self.path_label, "EXP")
            hf.attrs["path_name"]      = IDX_TO_NAME.get(self.path_label, "unknown")
            hf.attrs["n_frames"]       = n
            hf.attrs["duration_s"]     = float(timestamps[-1])
            hf.attrs["record_hz"]      = self.record_hz
            hf.attrs["created_at"]     = datetime.now().isoformat()
            hf.attrs["obs_fields"]     = ",".join(obs_names)

            hf.create_dataset("timestamps",   data=timestamps,   compression="gzip")
            hf.create_dataset("motor_angles", data=motor_angles, compression="gzip")
            hf.create_dataset("actions",      data=actions,      compression="gzip")
            hf.create_dataset("task_labels",  data=task_labels,  compression="gzip")
            hf.create_dataset("obs",          data=obs,          compression="gzip")

    def _save_npz(self, fpath: str):
        n = len(self._frames)
        obs_names = [name for name, _, _ in OBS_FIELDS]
        obs = np.zeros((n, len(obs_names)), dtype=np.float32)
        for i, f in enumerate(self._frames):
            fd = asdict(f)
            for j, name in enumerate(obs_names):
                obs[i, j] = fd.get(name, 0.0)

        np.savez_compressed(
            fpath,
            timestamps=np.array([f.timestamp for f in self._frames], dtype=np.float32),
            motor_angles=np.array([[f.m0_angle, f.m1_angle, f.m2_angle] for f in self._frames], dtype=np.float32),
            actions=np.array([[f.delta_m0, f.delta_m1, f.delta_m2] for f in self._frames], dtype=np.float32),
            task_labels=np.array([f.task_label for f in self._frames], dtype=np.int8),
            obs=obs,
            path_label=np.array([self.path_label]),
        )

    # ── 工具 ──────────────────────────────────────────────────

    def print_stats(self):
        if not self._frames:
            print("[DataCollector] 无帧数据")
            return
        n   = len(self._frames)
        dur = self._frames[-1].timestamp
        nav = sum(1 for f in self._frames if f.task_label == TASK_NAVIGATE)
        clr = sum(1 for f in self._frames if f.task_label == TASK_CLEAR)
        dm0 = np.std([f.delta_m0 for f in self._frames])
        dm1 = np.std([f.delta_m1 for f in self._frames])
        dm2 = np.std([f.delta_m2 for f in self._frames])
        print(f"[DataCollector] 统计: {n} 帧  {dur:.1f}s  {n/dur:.1f}fps")
        print(f"  导航帧={nav}  清理帧={clr}")
        print(f"  动作标准差: Δm0={dm0:.3f}°  Δm1={dm1:.3f}°  Δm2={dm2:.3f}°")

    @property
    def is_recording(self) -> bool:
        return self._is_recording

    @property
    def frame_count(self) -> int:
        return len(self._frames)

    @property
    def elapsed_time(self) -> float:
        if self._start_time is None:
            return 0.0
        return time.time() - self._start_time


# ──────────────────────────────────────────────────────────────
# 工具：列出已录制的演示文件
# ──────────────────────────────────────────────────────────────

def list_demos(demo_dir: str = "expert_demos") -> List[dict]:
    """列出 demo_dir 下所有有效的演示文件及其元信息。"""
    import json as _json
    result = []
    if not os.path.exists(demo_dir):
        return result

    for fname in sorted(os.listdir(demo_dir)):
        fpath = os.path.join(demo_dir, fname)

        if fname.endswith(".json"):
            try:
                with open(fpath, "r", encoding="utf-8") as fp:
                    d = _json.load(fp)
                result.append({
                    "file":       fpath,
                    "format":     "json",
                    "session":    d.get("session_name", ""),
                    "yolo_code":  d.get("yolo_code", "?"),
                    "path_label": int(d.get("path_label", 0)),
                    "path_name":  d.get("path_name", ""),
                    "n_frames":   int(d.get("n_frames", 0)),
                    "duration_s": float(d.get("duration_s", 0)),
                })
            except Exception:
                pass

        elif fname.endswith(".h5") and HDF5_AVAILABLE:
            try:
                with h5py.File(fpath, "r") as hf:
                    result.append({
                        "file":       fpath,
                        "format":     "hdf5",
                        "session":    hf.attrs.get("session_name", ""),
                        "yolo_code":  hf.attrs.get("yolo_code", "?"),
                        "path_label": int(hf.attrs.get("path_label", 0)),
                        "path_name":  hf.attrs.get("path_name", ""),
                        "n_frames":   int(hf.attrs.get("n_frames", 0)),
                        "duration_s": float(hf.attrs.get("duration_s", 0)),
                    })
            except Exception:
                pass

        elif fname.endswith(".npz"):
            try:
                d = np.load(fpath)
                result.append({
                    "file":       fpath,
                    "format":     "npz",
                    "path_label": int(d["path_label"][0]),
                    "n_frames":   len(d["timestamps"]),
                    "duration_s": float(d["timestamps"][-1]),
                })
            except Exception:
                pass

    return result


if __name__ == "__main__":
    print("支气管路径定义:")
    for k, v in BRONCHUS_PATHS.items():
        print(f"  {k}: {v}")
    print(f"\n观测向量维度: {len(OBS_FIELDS)}")
    for name, scale, desc in OBS_FIELDS:
        print(f"  {name:20s} ÷{scale:6.1f}  {desc}")
