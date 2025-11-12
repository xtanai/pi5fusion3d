# Pi5Fusion3D

**Multi‑stereo fusion and calibration engine for precise 3D hand/gesture key‑poses.**

Pi5Fusion3D consumes **ROI/keypoint streams** from multiple **Pi5Track3D** stereo rigs over LAN, performs **time alignment, multi‑view calibration, bundle adjustment, and low‑latency fusion**, and publishes a clean **3D key‑pose stream** to downstream clients such as **MotionCoder**.

> **Status:** early prototype (WIP). Interfaces may change.

---

## Why Pi5Fusion3D?

* **Scale:** Combine **2–4 stereo pairs** (4–8 cameras) for robust, occlusion‑resistant tracking.
* **Precision:** Multi‑view geometry + **AprilTag/wrist/fingertip references** yields stable **Z‑scale** and mm‑level repeatability.
* **Determinism:** Integrates with **TDMStrobe** phase metadata (A/B/C/D) for cross‑illumination control.
* **Latency:** Lightweight fusion path (keypoints/ROI only), GPU‑accelerated triangulation & filtering.
* **Separation of concerns:** Keep capture on the edge (**Pi5Track3D**); do heavy fusion and configuration in one place (this host).

---

## System Architecture

```
[TDMStrobe] ── TRIG A/B ─► [Pi5Track3D #1..#N] ── LAN ─► [Pi5Fusion3D] ──► MotionCoder / Apps
                                 │                                  │
                             RAW10→ROI,                       3D key‑poses
                        tags/wrist/fingertips                 (joints+conf)
```

* **Inputs:** per‑rig streams (keypoints, ROI crops or sparse point clouds, reference features, timestamps, TDM phase IDs).
* **Core:** timebase alignment, outlier rejection, **per‑rig + global calibration**, triangulation, bundle adjust, temporal filtering.
* **Outputs:** unified **3D key‑poses** (with confidences & references), optional diagnostics/metrics.

---

## Features

* **Multi‑rig ingestion**: 3–4 stereo rigs over UDP/ZeroMQ/TCP.
* **Time alignment**: monotonic clock sync + per‑rig latency compensation.
* **Calibration tools**:

  * Intrinsics/extrinsics from **AprilTag L‑brackets** or Charuco panels.
  * One‑click **bundle adjustment** with robust loss and priors (Tag board, wristband, fingertip aids).
* **Fusion & filtering**:

  * Triangulation with uncertainty propagation.
  * Outlier culling (RANSAC/Huber/Tukey), temporal smoothing (Savitzky–Golay/One‑Euro).
  * Confidence gating & joint‑level fallbacks.
* **Low‑latency path**: ROI‑only option; avoids raw video transport.
* **Live monitoring**: web UI for per‑rig status, histograms, residual plots, and frame phase.
* **Exports**: stream to **MotionCoder**, CSV/JSON logs, optional ROS topic.

---

## I/O Schemas

### Inbound (from Pi5Track3D)

```json
{
  "rig_id": "rig01",
  "frame_id": 1245678,
  "timestamp_ns": 1731400123456789,
  "phase": "A",                      // TDM phase (A/B/C/D)
  "keypoints_2d": {
    "hand": [[x,y,conf], ...],        // normalized or pixel coords
    "wrist": [x,y,conf],
    "fingertips": [[x,y,conf], ...]
  },
  "roi_points3d": [[X,Y,Z,conf], ...], // optional sparse point cloud in rig frame
  "refs": {
    "apriltag_corners": [[x,y,id],...],
    "board_id": "LBRACKET_FRONT"
  },
  "intrinsics": {"fx":..., "fy":..., "cx":..., "cy":..., "k": [...]},
  "extrinsics": {"R": [...], "t": [...]}
}
```

### Outbound (to MotionCoder / clients)

```json
{
  "pose3d": {
    "joints": [
      {"name":"wrist","X":... ,"Y":...,"Z":...,"conf":...},
      {"name":"thumb_tip", ...},
      {"name":"index_tip", ...}
    ],
    "frame_id": 1245678,
    "timestamp_ns": 1731400123456789,
    "rigs_used": ["rig01","rig02","rig03"],
    "quality": {"rms_reproj_px": 0.41, "num_inliers": 56}
  },
  "refs": {"apriltag_board": "LBRACKET_FRONT", "wristband": true},
  "meta": {"phase":"A","latency_ms": 7.8}
}
```

---

## Requirements

* **Host OS:** Linux (Ubuntu 22.04+ recommended)
* **GPU:** NVIDIA CUDA‑capable (e.g., RTX 3060/3080/3090); CPU‑only mode available with reduced throughput
* **Network:** Gigabit Ethernet (low‑jitter preferred)
* **Dependencies:**

  * Core: C++/Python, **Eigen**, **ceres‑solver** (BA), **OpenCV**, **Sophus**
  * Optional: **PyTorch** (MMPose), **Anipose**
  * Messaging: **ZeroMQ**/**UDP**/**TCP** (select at build/runtime)

---

## Quick Start

1. **Install deps** (`ceres`, `opencv`, `zeromq`, CUDA optional).
2. Launch **Pi5Track3D** on each rig; verify they publish LAN packets.
3. Create a **config.yaml** (see below) and run `pi5fusion3d --config config.yaml`.
4. Open the **web UI** to confirm rig clocks, phase usage, and residuals.
5. Run **calibration** (tag solve + BA). Save the **global extrinsics**.
6. Start the **Pose stream** → subscribe from **MotionCoder**.

---

## Configuration Example (`config.yaml`)

```yaml
fusion:
  rigs: [rig01, rig02, rig03, rig04]
  transport: zeromq   # udp|tcp|zeromq
  max_latency_ms: 30
  outlier: { method: huber, delta: 1.0 }
  smoothing: { method: one_euro, min_cutoff: 1.0, beta: 0.05 }
  output:
    fps: 120
    endpoint: tcp://0.0.0.0:5557

rigs:
  rig01:
    subscribe: tcp://rig01.local:5555
    time_offset_ms: 0
    weight: 1.0
  rig02:
    subscribe: tcp://rig02.local:5555
    time_offset_ms: -2.3
    weight: 1.0
  rig03:
    subscribe: tcp://rig03.local:5555
    time_offset_ms: 1.1
    weight: 0.9

calibration:
  tag_family: apriltag_36h11
  board: LBRACKET_FRONT
  optimize: { intrinsics: false, extrinsics: true, scale: true }
  priors:
    wristband: true
    fingertips: true
```

---

## CLI

```
pi5fusion3d --config config.yaml \
            --log-level info \
            --web-ui 0.0.0.0:8080 \
            --record /data/sessions/run_001
```

**Subcommands**

* `calibrate`: solve tag board → init extrinsics → run BA
* `bench`: synthetic timing/latency & throughput test
* `replay`: ingest recorded LAN stream for offline tuning

---

## Performance Notes

* Favor **RAW10→ROI** on the edge to minimize bandwidth.
* Keep **phase consistency** (TDM A/B/C/D) across rigs to avoid cross‑lit frames.
* For 120 fps, target an **end‑to‑end latency < 10 ms** (host‑only path) on RTX‑class GPUs.

---

## Roadmap

* Per‑rig **latency auto‑cal** via cross‑correlation.
* Multi‑rig **auto‑scale alignment** using redundant tag boards.
* IMU fusion hooks (wrist/hand) and per‑joint uncertainty output.
* ROS 2 bridge; gRPC streaming API.
* GUI tools for tag board layout & calibration reports.

---

## License

**Apache‑2.0** (code, docs). Calibration boards/mechanics may use **CERN‑OHL‑S**.

---

## Safety

Pi5Fusion3D processes data only, but it assumes a capture setup using **NIR illumination**. Follow the project’s **Safety** guidance (IEC 62471, baffling, interlocks). When in doubt, reduce pulse width/duty and shield emitters.
