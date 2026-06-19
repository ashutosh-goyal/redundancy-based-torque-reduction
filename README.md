# Redundancy-Based Torque Reduction for Manipulators


Robots that work alongside humans increasingly rely on **kinematic redundancy** — extra degrees of freedom beyond what's strictly needed for a task — to handle secondary objectives like obstacle avoidance and compliant behavior. This repository implements a redundancy-resolution strategy that takes inspiration from how weightlifters distribute load across joints and muscles to avoid injury: instead of letting external forces concentrate torque on a single joint, the robot continuously reconfigures itself in its null space to **spread the load and avoid torque saturation**, all while tracking the exact same end-effector trajectory.

Validated on a 7-DOF KUKA LBR iiwa 14 R820 under live external force perturbation, the method achieves a **29.85% reduction in peak torque** and a **14.69% reduction in cumulative torque** during trajectory tracking, with no change to the commanded Cartesian path.

---

## 🎥 Demo

A side-by-side comparison of the robot tracking a circular trajectory under external force, with and without redundancy optimization:



https://github.com/user-attachments/assets/37490ce3-8a50-4a1d-bb33-5478c2cfc1c5

*Left: baseline configuration (no redundancy exploitation) — torque concentrates on fewer joints.*
*Right: optimized configuration — null-space motion redistributes load, visibly lowering joint torques.*

---

## 🧠 Core Idea

Most prior approaches to torque-aware redundancy resolution fall into one of two camps:

| Approach | Limitation |
|---|---|
| **Cartesian stiffness control** ([Ajoudani et al.](https://doi.org/10.1109/TRO.2017.2750697)) | Tightly coupled to a specific controller; only aligns the stiffness ellipsoid, can't shape arbitrary ellipsoids (e.g. for polishing tasks); ignores moments |
| **Kinetic energy minimization** | Numerically unstable over long horizons |

This work instead proposes a **controller-agnostic, trajectory-level** objective function that:

1. Considers **both forces and moments** as a combined 6D wrench at the end-effector — not forces alone.
2. Produces a **trajectory** (joint positions/velocities) rather than a torque command, so it can be paired with any downstream controller (position, velocity, impedance, etc.).
3. Is rooted in aligning the manipulator's **force ellipsoid** (from $JJ^T$) with the direction of the external perturbation, which provably minimizes the torque needed to resist or generate that wrench.

### The objective function

A force–moment perturbation matrix is built from the sensed/estimated external wrench:

```
K = diag(|Fx|, |Fy|, |Fz|, |Mx|, |My|, |Mz|)
```

and compared against the manipulator's configuration-dependent force ellipsoid $JJ^T$ via a normalized Frobenius distance:

```
W = || JJᵀ/tr(JJᵀ) − K/tr(K) ||_F
```

**Maximizing** $W$ via gradient ascent in the manipulator's null space:

```
q̇_opt = G · ∂W/∂q
```

drives the principal axis of the force ellipsoid into alignment with the external perturbation — the configuration that requires the *least* torque to resist or produce that wrench. This update is injected purely through the null-space projector, so the commanded end-effector trajectory is never disturbed:

```
q(t+Δt) = q(t) + J†(ẋ_d + Kp·e(t))·Δt + (I − J†J)·q̇_opt
```

A worked 2-link planar example in the paper shows this clearly: for an identical applied force, some configurations need **zero** joint torque to counteract it, while a singular configuration aligned the wrong way demands the *maximum* torque — the objective function $W$ peaks exactly at the zero-torque configurations.

External wrenches are estimated on-line from joint torque residuals via Tikhonov-regularized inversion of the Jacobian:

```
P̂(t) = (JᵗJ + ηI)⁻¹ Jᵗ τ̂_ext(t)
```

---

## 🔬 Experimental Setup

| Parameter | Value |
|---|---|
| Robot | KUKA LBR iiwa 14 R820 (7-DOF) |
| Task | Circular trajectory, radius 0.2 m, Y–Z plane, 40 s period |
| Redundancy exploited | 4 DOF (task constrained to Cartesian position only) |
| External force | 3 N, applied at end-effector |
| Proportional gain | $K_p = 10\,I_3$ |
| Null-space gain | $G = 100$ |
| Time step | $\Delta t = 1/100$ s |

### Results

| Metric | Improvement |
|---|---|
| Peak joint torque | **−29.85%** |
| Cumulative joint torque | **−14.69%** |
| Max trajectory tracking error | 0.0075 m (on a 0.2 m radius circle) |

The small increase in tracking error under optimization comes from the velocity controller's response to the added null-space motion, and can be reduced by shrinking $\Delta t$ or using a less aggressive gain $G$.

---

## 📁 Repository Structure

```
.
├── media/                  # Demo video(s), figures, plots
├── src/                    # Implementation
│   ├── objective_function.py   # W(q), ∂W/∂q
│   ├── redundancy_resolution.py# Null-space projection + IK integration (Eq. 8)
│   ├── disturbance_estimation.py # Tikhonov-regularized wrench estimation
└── README.md
```


---

---

## 📖 Citation

If you use this work, please cite:

```bibtex
@article{jadav2024redundancy,
  title     = {Utilization of Manipulator Redundancy for Torque Reduction During Force Interaction},
  author    = {Jadav, Shail and Palanthandalam-Madapusi, Harish J.},
  journal   = {ASME Letters in Dynamic Systems and Control},
  volume    = {4},
  number    = {2},
  pages     = {021005},
  year      = {2024},
  publisher = {American Society of Mechanical Engineers},
  doi       = {10.1115/1.4064654}
}

## 📄 License

This implementation is released under the [MIT License](LICENSE). The associated paper is © 2024 ASME; please respect the journal's copyright when reproducing figures or text from the publication itself.
