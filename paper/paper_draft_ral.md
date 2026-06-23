# An Embodied Autonomous Bronchoscope Platform with Multimodal Perception and Goal-Conditioned Imitation Learning

> **Target journal:** IEEE Robotics and Automation Letters (RA-L)  
> **Page limit:** 6 pages + references  
> **Status:** Draft v0.1 — fill in experiment data before submission

---

## Abstract (≤150 words — write last)

We present an embodied autonomous bronchoscope platform that integrates multimodal perception with goal-conditioned imitation learning for autonomous airway navigation and mucus clearance. The system combines real-time semantic segmentation (UNet), metric depth estimation (Depth-Anything-V2, ViT-L), and depth-guided bifurcation analysis into a 20-dimensional structured observation space. A goal-conditioned GRU policy network (BronchusPolicy) maps multimodal observations and a target bronchial-path identifier to 3-DoF motor increments, enabling navigation to any of nine anatomical destinations via voice command. Expert demonstration data collected at 20 Hz in an HDF5 format are used to train the policy through behavior cloning with an auxiliary subtask classification head. Experiments on a bronchial phantom [and ex-vivo porcine lung] demonstrate a navigation success rate of **XX%** and a mean positioning error of **XX mm**, at an inference throughput of 20 Hz on a consumer GPU. The complete platform—hardware, perception, policy, and user interface—runs as a unified system, providing a clinically deployable foundation for autonomous bronchoscopy.

**Keywords:** bronchoscopy, autonomous surgical robot, imitation learning, depth estimation, multimodal perception, goal-conditioned policy

---

## I. Introduction

Flexible bronchoscopy is a critical clinical procedure for airway inspection, biopsy sampling, and mucus clearance in patients with respiratory disease. Manual bronchoscopic navigation requires years of operator training to develop spatial awareness of complex branching geometry and to coordinate instrument tip deflection under real-time endoscopic video. Operator fatigue and inter-clinician variability remain significant concerns, particularly in intensive-care settings where repeated bronchial toilet (suction lavage) is required.

Robotic bronchoscopy has attracted considerable research attention [cite: Camarillo2008, Swaney2017, Roesthuis2013]. Existing commercial platforms (e.g., Monarch, Ion) provide teleoperation assistance but do not offer autonomous waypoint navigation through the full bronchial tree. Key open challenges include: (1) robust real-time lumen segmentation and depth perception in textureless, wet endoscopic video; (2) bifurcation-aware path selection under anatomical variability; and (3) learning-based policies that generalize across patients without explicit kinematic models.

Recent advances in vision foundation models—particularly large-scale monocular depth networks [cite: Yang2024DepthAnythingV2] and encoder–decoder segmentation architectures—have enabled per-frame metric depth estimation and semantic parsing at interactive frame rates. Concurrently, imitation learning (behavior cloning, BC) combined with temporal recurrent architectures has demonstrated end-to-end visuomotor policies for manipulation [cite: Chi2023DiffusionPolicy, Zhao2023ACT] and flexible-endoscope navigation [cite: Xu2023, Li2022].

In this letter, we make the following contributions:

1. **Integrated multimodal perception pipeline**: UNet-based lumen segmentation, Depth-Anything-V2 metric depth, and a depth-region analysis module (DepthPathFinder) are fused into a 20-dimensional structured observation vector that captures lumen geometry, bifurcation topology, and instrument proprioception.

2. **Goal-conditioned GRU policy (BronchusPolicy)**: A temporal recurrent network with a goal-embedding mechanism maps multimodal observations and an anatomical target identifier to 3-DoF motor increments (Δm0, Δm1, Δm2), supporting goal-directed navigation to nine anatomical destinations via an integer goal token.

3. **Dual-head architecture with subtask classification**: An auxiliary task-classification head distinguishes navigation from mucus-clearance phases, improving action quality in both regimes under a multi-task training objective.

4. **Full system integration and real-time deployment**: The platform unifies hardware control, perception, policy inference, expert-data collection (HDF5, 20 Hz), and a voice-command interface into a single deployable system validated on [phantom / ex-vivo porcine lung].

---

## II. System Overview

Fig. 1 shows the overall architecture. The system consists of four layers: (a) the **hardware layer** (3-DoF tendon-driven bronchoscope actuated by RMD motors); (b) the **perception layer** (UNet segmentation + Depth-Anything-V2 depth + DepthPathFinder); (c) the **policy layer** (goal-conditioned BronchusPolicy, trained offline via BC); and (d) the **interaction layer** (Tkinter GUI, Xbox-controller teleoperation, Baidu-ASR voice commands, ActionRecorder for data collection).

The system operates in four modes managed by a finite-state machine: *Manual* (joystick), *Vision* (reactive visual servoing), *Hybrid* (manual + visual assist), and *AI-Auto* (autonomous policy). Mode transitions are triggered by GUI buttons or voice commands (e.g., "切换为自主模式").

---

## III. Hardware Platform

### A. Robotic Bronchoscope

The prototype uses a commercial flexible bronchoscope with a 4-way deflectable tip, retrofitted with three RMD brushless motors:

| DoF | Motor | Range | Gearbox | Function |
|-----|-------|--------|---------|----------|
| M0 | RMD-X | [−900°, 0°] | 1:1 | Scope advancement / withdrawal |
| M1 | RMD-X | [−170°, 170°] | 1:1 | Left–right tip deflection |
| M2 | RMD-X | [−500°, 500°] | 10:1 | Up–down tip deflection |

Motors are controlled over a UART serial bus at 1 Mbaud. Angle commands are issued as position targets with velocity limits; the control loop runs at **20 Hz**. M2 uses a 10:1 reduction gearbox (M2\_DELTA\_MULTIPLIER = 10), so the effective output range corresponds to [−50°, 50°] tip deflection.

### B. Sensor Suite

- **Endoscope camera**: [model], 640×480, 30 fps, USB 3.0; circular ROI of 788 px diameter extracted and barrel-distortion corrected offline.
- **Joystick**: Xbox controller via XInput for manual teleoperation.
- **Microphone**: for Baidu cloud-ASR (Chinese language) voice commands.

---

## IV. Multimodal Perception

### A. Semantic Segmentation (UNet)

A UNet with a VGG-16 backbone, fine-tuned on an in-house bronchoscopy dataset of **N** annotated frames, performs per-pixel classification of four classes: *background*, *lumen*, *bifurcation*, and *mucus/stone*. Inference runs on GPU at ≈30 fps. The predicted probability maps yield:

- **Stone / mucus centroid** (cx, cy): pixel centroid of the mucus class, normalized to [−1, 1] w.r.t. the ROI.
- **Stone area ratio**: fraction of ROI pixels classified as mucus.
- **Bifurcation centroid** (bfx, bfy) and **area ratio**: analogous quantities for the bifurcation class.

### B. Metric Depth Estimation (Depth-Anything-V2)

Depth-Anything-V2 [cite: Yang2024] with a ViT-L encoder and metric-depth head (Hypersim fine-tuned weights) estimates per-pixel absolute depth in millimeters. Input: the barrel-corrected ROI, resized to 518×518. Output: depth map at native resolution, processed at **≈15 fps** on an RTX 3090.

Depth values outside the valid range [Z\_MIN=5 mm, Z\_MAX=300 mm] are masked. A jump-suppression filter discards frames where the mean depth changes by more than Z\_JUMP=30 mm between consecutive frames. From the filtered map we extract:

- **depth\_center\_mm**: depth at image center (scope axis).
- **depth\_mean\_mm**: mean depth inside the circular lumen mask.
- **depth\_max\_mm**: 95th-percentile depth (deepest navigable region).

### C. DepthPathFinder — Bifurcation and Path Analysis

At bifurcation zones, the depth map is partitioned into a 3×5 grid. Each column (left / center / right) accumulates depth statistics over the distal half of the lumen mask, yielding three scalar path-depth estimates:

- **path\_left\_mm**, **path\_center\_mm**, **path\_right\_mm**

These are passed directly to the observation vector, allowing the policy to reason about available paths without an explicit 3-D map. In the reactive *Vision* mode, the deepest path drives a proportional visual-servo controller; in *AI-Auto* mode, the GRU policy uses all three values simultaneously.

### D. Structured Observation Vector

All perception outputs and motor proprioception are assembled into a **20-dimensional** structured observation vector $\mathbf{o}_t$:

$$
\mathbf{o}_t = \underbrace{[\text{cx, cy, area, det, bfx, bfy, bfa}]}_{\text{visual: 7}} \;
\| \;
\underbrace{[\text{d\_ctr, d\_mean, d\_max, p\_l, p\_c, p\_r}]}_{\text{depth: 6}} \;
\| \;
\underbrace{[m_0, m_1, m_2]}_{\text{position: 3}} \;
\| \;
\underbrace{[\dot{m}_0, \dot{m}_1, \dot{m}_2]}_{\text{velocity: 3}} \;
\| \;
\underbrace{[\Delta t]}_{\text{timing: 1}}
$$

Each field is normalized by a fixed scale factor (Table I). Velocity $\dot{m}_i$ is estimated as the finite difference of consecutive angle readings divided by $\Delta t$.

---

## V. Goal-Conditioned Policy Learning

### A. Policy Architecture (BronchusPolicy)

Given a sliding window of $T=8$ consecutive observations $\{\mathbf{o}_{t-T+1}, \ldots, \mathbf{o}_t\}$ and a discrete goal token $g \in \{0,\ldots,9\}$ indexing one of ten anatomical targets (Table II), BronchusPolicy outputs:

1. **Action** $\mathbf{a}_t \in [-1,1]^3$: normalized motor increments $[\Delta m_0, \Delta m_1, \Delta m_2]$, scaled to [±5°, ±3°, ±3°]/frame respectively.
2. **Subtask logits** $\mathbf{z}_t \in \mathbb{R}^2$: classification of the current frame as *navigate* (0) or *clear-mucus* (1).

The network comprises three modules (Fig. 2):

**ObsEncoder**: Two-layer MLP (OBS\_DIM → 128 → 128) with LayerNorm and ELU activations encodes each frame independently.

**Goal Embedding**: An embedding table $E \in \mathbb{R}^{10 \times 16}$ maps the integer goal token to a 16-dim vector, broadcast across the time dimension.

**Temporal GRU**: A 2-layer GRU (hidden size 256) processes the concatenated sequence $[\text{enc}(\mathbf{o}_t) \| \mathbf{e}_g]_{t=1}^T$ of dimension 144. The last hidden state $\mathbf{h}_T$ feeds two heads:

- *Action head*: MLP (256 → 128 → 3) + Tanh.
- *Task head*: MLP (256 → 64 → 2).

Orthogonal weight initialization is applied throughout. Total parameters: **~0.4 M**.

### B. Expert Demonstration Collection

Expert demonstrations are recorded during manual teleoperation at **20 Hz** using `BronchusDataCollector`. Each frame is stored in an HDF5 file with fields matching the 20-dim observation vector plus: (i) the raw motor angle triplet $[m_0, m_1, m_2]$, (ii) a task label (0: navigate, 1: clear-mucus), and (iii) a path label $g$ indicating the intended bronchial destination.

Target dataset: ≥80 trajectories, ≥4 h total, covering straight advancement, left/right bifurcation turns, mucus-clearance episodes, and withdrawal.

### C. Behavior Cloning Training

The training objective combines an MSE action-regression loss and a cross-entropy subtask-classification loss:

$$
\mathcal{L} = \underbrace{\|\mathbf{a}_t - \hat{\mathbf{a}}_t\|_2^2}_{\mathcal{L}_{\text{action}}} + \lambda \underbrace{H(\mathbf{z}_t, y_t)}_{\mathcal{L}_{\text{task}}}
$$

with $\lambda=0.3$. Training uses the AdamW optimizer, cosine annealing LR schedule (initial $3\times10^{-4}$), batch size 64, sequence length $T=8$, mixed-precision (AMP), and early stopping with patience 30. The dataset is split 90/10 train/validation.

### D. Real-Time Inference

At test time, `BCRunner` maintains a rolling FIFO buffer of $T=8$ frames. Each control tick:
1. The perception pipeline writes fresh values to a thread-safe feature dictionary.
2. The buffer is updated and packed into a $[1, T, 20]$ tensor.
3. A forward pass through BronchusPolicy produces action and subtask prediction.
4. The action is scaled and applied via position-control commands to the three motors.

Measured inference latency: **<5 ms** on RTX 3090, sustaining **20 Hz** end-to-end.

---

## VI. Experiments

> **[PLACEHOLDER — complete before submission]**

### A. Experimental Setup

Experiments are conducted on: (1) a 3D-printed bronchial phantom (main carina + left/right principal bronchi + upper/lower lobes); (2) [optionally] ex-vivo porcine lung mounted in a torso fixture.

Metrics:
- **Navigation Success Rate (NSR)**: fraction of trials in which the scope reaches the target bronchus without wall collision within the maximum step budget.
- **Mean Endpoint Error (MEE)**: Euclidean distance (mm) between the scope tip and the ground-truth target centroid at trial termination.
- **Inference Latency**: wall-clock time per control cycle.
- **Trajectory Smoothness**: mean jerk of motor-angle sequences.

### B. Baselines

| Method | Description |
|--------|-------------|
| Manual (expert) | Experienced operator, no assistance |
| Vision (reactive) | DepthPathFinder + visual servoing, no learned policy |
| BC (ours, no goal) | BronchusPolicy with goal token fixed to EXP (exploration) |
| BC+goal (ours) | Full BronchusPolicy with correct goal token |

### C. Results

*[Insert Table III: NSR, MEE, Latency across methods and targets]*

*[Insert Fig. 3: representative trajectory overlays on phantom image]*

Key findings:
- BC+goal achieves **XX% NSR** vs. **XX%** for reactive vision baseline, demonstrating the benefit of goal conditioning.
- The dual-head architecture improves action MSE by **XX%** compared to a single-head ablation.
- 20 Hz end-to-end operation is maintained on hardware.

---

## VII. Conclusion

We presented an integrated embodied bronchoscope platform that combines multimodal perception (UNet segmentation, Depth-Anything-V2 metric depth, DepthPathFinder) with a goal-conditioned GRU policy trained via behavior cloning. The 20-dimensional structured observation design bridges visual perception and motor proprioception, while the dual-head architecture simultaneously models navigation and mucus-clearance behaviors. Real-time deployment at 20 Hz was demonstrated on [hardware]. Future work will extend the policy with PPO fine-tuning in a PyBullet bronchial simulator and validate sim-to-real transfer on ex-vivo animal tissue.

---

## References

[1] D. C. Rucker et al., "Deflection-Based Force Sensing for Continuum Robots," IROS 2011.  
[2] P. Swaney et al., "Toward Robotic Bronchoscopy," JMRR 2017.  
[3] L. Yang et al., "Depth Anything V2," NeurIPS 2024.  
[4] T. Chi et al., "Diffusion Policy," RSS 2023.  
[5] T. Zhao et al., "Learning Fine-Grained Bimanual Manipulation with Low-Cost Hardware," RSS 2023.  
[6] [Add bronchoscopy navigation / imitation learning references as needed]

---

## Supplementary

- Demo video: autonomous navigation to left/right principal bronchi in phantom.
- HDF5 data format specification.
- Code: [GitHub link TBD].
