# From localization error to an inferred effective wheel separation

## Finding and investigation

The earlier experiment produced a clean wheel estimate close to the planned `(1, 1)` endpoint, yet the simulated robot ended elsewhere. Its clean heading was approximately 45 degrees while Gazebo truth was approximately 51 degrees. The discrepancy existed before the straight-driving phase, so an apparently successful wheel-angle execution did not mean the physical robot followed the expected heading.

This article follows the evidence from an anomalous localization result to a turning-scale diagnosis and an inferred effective wheel separation. The companion [collision geometry article](Differential_Drive_Wheel_Contact_Diagnosis_and_Fix.md) explains the model change and the recorded comparison after it.

## 1. Data and quantities being compared

The source is [the archived earlier CSV](../results/old/localization_20261009_230014_130403.csv), containing **1,457 matched samples**. The record contains clean/noisy wheel estimates and independently published Gazebo poses matched by simulation timestamp.

- **Clean estimate:** wheel-angle integration without added measurement noise.
- **Noisy estimate:** the same wheel increments with injected Gaussian errors.
- **Ground truth:** the independently published simulated robot pose, used only for evaluation.

The current model and geometry configuration agree on wheel radii of **0.025 m** and centre-to-centre wheel separation of **0.10 m**. The archived record belongs to the earlier single-goal experiment, not the current three-waypoint report dataset. The CSV itself does not store a full configuration snapshot.

### Source columns and calculation rules

The diagnosis reads these columns from the fixed earlier CSV:

| CSV columns | Use |
|---|---|
| `time_s` | Simulation time of each matched row |
| `x_clean_m`, `y_clean_m` | Clean endpoint and first displacement above 0.01 m |
| `x_gt_m`, `y_gt_m` | Independent true position and clean position error |
| `theta_clean_rad`, `theta_gt_rad` | Heading checkpoints, accumulated turns, and effective separation |
| `clean_error_m` | Clean position RMSE and final clean error |
| `position_error_m` | Noisy position RMSE and final noisy error |

```text
clean_error[i] = hypot(x_clean_m[i] - x_gt_m[i], y_clean_m[i] - y_gt_m[i])
clean_RMSE = sqrt(sum(clean_error_m[i]^2) / 1457)
noisy_RMSE = sqrt(sum(position_error_m[i]^2) / 1457)
checkpoint_row = argmin(abs(theta_clean_rad[i] - radians(requested_angle)))
clean_turn = wrap(theta_clean_rad[last] - theta_clean_rad[first])
true_turn = wrap(theta_gt_rad[last] - theta_gt_rad[first])
```

Heading values are stored in radians; convert to degrees for presentation. Use full precision in calculations. The wrapped turn formulas apply to this short single-turn record. The nominal `b = 0.10 m` and original collision width `0.012 m` come from model configuration/history, not measurements stored in the CSV.

## 2. First warning sign from position error

| Quantity | Earlier recorded value |
|---|---:|
| Clean position RMSE | 0.082983 m |
| Noisy position RMSE | 0.086259 m |
| Final clean position error | 0.149262 m |
| Final noisy position error | 0.164427 m |
| Final clean position | (0.999396, 1.000800) m |
| Final ground-truth position | (0.887775, 1.099895) m |

The clean estimate almost reaches `(1, 1)`, but it differs from truth by approximately **14.93 cm**. A clean RMSE of **8.30 cm** also exists without added noise. This is why the investigation cannot stop at increasing or decreasing the noise settings: the disagreement is already present in the wheel-motion baseline.

The difference between noisy and clean RMSE is not a decomposition of the total error. Random errors can add to or cancel other errors, and RMSE is nonlinear. The clean trajectory provides the clearer starting point for investigating geometry and motion execution.

## 3. Locate when the heading discrepancy appears

Define the diagnostic difference as:

```text
heading_difference = wrap(theta_ground_truth - theta_clean)
```

A positive value means the simulated body turned farther anticlockwise than the clean wheel estimate. This is a separate quantity from the CSV's `heading_error_rad`, which compares the **noisy** estimate with truth in the opposite subtraction order.

To identify a checkpoint near the beginning of straight travel, select the first clean position more than **0.01 m** from its initial position. This is an analysis threshold, not the controller's arrival tolerance or an exact phase-transition timestamp.

| Checkpoint | Simulation time | Truth minus clean heading |
|---|---:|---:|
| First clean displacement above 0.01 m | 7.84 s | 6.054528967 degrees |
| Final sample | 20.10 s | 6.054528902 degrees |
| Additional discrepancy between these samples | — | approximately -0.000000065 degrees |

The difference is already approximately **6.05 degrees** when meaningful straight travel begins and changes negligibly afterwards. For this record, the persistent discrepancy principally develops during turning, rather than accumulating as further heading drift during straight travel.

## 4. Check whether the turn error is an offset or a scale mismatch

For each nominal clean heading, select the CSV sample whose clean heading is closest to that value. The actual sample heading is retained below rather than treating the nominal label as an exact measurement.

| Nominal clean checkpoint (degrees) | Time (s) | Actual clean heading (degrees) | Truth heading (degrees) | Truth minus clean (degrees) |
|---|---:|---:|---:|---:|
| 0 | 5.54 | 0.000000 | 0.000000 | 0.000000 |
| 15 | 6.18 | 14.954198 | 16.930513 | 1.976315 |
| 30 | 6.70 | 29.851101 | 33.858575 | 4.007474 |
| 45 | 7.46 | 45.008704 | 51.082823 | 6.074120 |

The initial headings are effectively zero. The discrepancy grows from approximately **1.98 degrees** near 15 degrees to **4.01 degrees** near 30 degrees and **6.07 degrees** near 45 degrees. That pattern is consistent with a rotational scale mismatch, rather than a fixed initial heading offset.

The near-45-degree checkpoint and final sample are different rows. Therefore, the checkpoint discrepancy of 6.074120 degrees and final discrepancy of 6.054529 degrees are both valid and should not be interchanged.

## 5. Reverse the wheel-motion equation

For measured wheel-angle increments:

```text
dl = rL * dphiL
dr = rR * dphiR
dtheta_clean = (dr - dl) / b_nominal
```

For an in-place turn, the wheel-travel difference `D = dr - dl` determines the predicted heading increment. If the same measured `D` is represented using an equivalent separation that explains the true accumulated turn:

```text
clean_turn = D / b_nominal
true_turn = D / b_effective
D = b_nominal * clean_turn = b_effective * true_turn
b_effective = b_nominal * clean_turn / true_turn
```

The analysis subtracts each trajectory's initial heading from its final heading. The initial headings in this run are negligible, so the accumulated turns are:

```text
clean_turn = 45.040216 degrees
true_turn = 51.094745 degrees
b_effective = 0.10 * 45.040216 / 51.094745
            = 0.08815039 m
            = 8.815039 cm
```

Degrees cancel in the ratio. The wheel-travel difference implied by the clean turn is approximately `0.10 * radians(45.040216) = 0.078610 m`. Dividing the same distance difference by the larger true turn yields the smaller effective separation.

The corresponding true-to-clean rotational scale is:

```text
scale = 51.094745 / 45.040216 = 1.134425
```

Thus, the simulated body turns approximately **13.44% farther** than the clean wheel model predicts for this recorded trial.

**What the inferred value means:** 0.08815039 m is the separation of an equivalent ideal differential-drive model that reproduces this turning scale. It is not a direct measurement of contact points or physical axle spacing. Slip, finite contact geometry, caster behaviour, wheel-radius assumptions, or timing errors could be absorbed into such a parameter. The actual wheel centres remain 0.10 m apart.

This derivation uses the net turn of this short single-turn record. A wrapped endpoint heading difference is not sufficient for trajectories containing full revolutions or multiple turns; those require accumulated, unwrapped heading increments.

## 6. Test whether the heading error explains the endpoint discrepancy

Under a simplified comparison, two paths start at the same point and travel equal distance `d`, with a direction difference `delta`. Their endpoint separation is:

```text
predicted_error = 2 * d * sin(abs(delta) / 2)
```

Using the planned straight distance `sqrt(2) m` and the final clean-to-truth heading discrepancy:

```text
predicted_error = 2 * sqrt(2) * sin(radians(6.054529) / 2)
                = 0.149372 m
recorded_final_clean_error = 0.149262 m
```

The prediction differs by approximately **0.000111 m** from the observed clean endpoint error. This close agreement supports a turning-heading explanation for this particular record. It is not a unique proof: the formula assumes equal starting points and equal travel distances, while actual motion may include translation during the turn, slip, and small integration errors.

Use the **clean** endpoint error for this comparison. The larger noisy endpoint error also contains the injected measurement uncertainty.

## 7. Connect the inferred separation to collision geometry

The original collision cylinders were **0.012 m wide**. With wheel centres separated by 0.10 m, the distance between their inner edges is:

```text
inner_edge_separation = 0.10 - 0.012 = 0.088 m
inferred_effective_separation = 0.08815039 m
absolute_difference = 0.00015039 m
```

The approximately 0.15 mm agreement motivates inspecting the wheel contact geometry. It does not show that the solver's actual contact points were exactly at the inner edges: the stored CSV has no contact-point or contact-force data.

The implemented response was to narrow the central collision band while keeping the physical wheel-centre spacing and wheel radius unchanged. The [companion article](Differential_Drive_Wheel_Contact_Diagnosis_and_Fix.md) documents that change and the later data.

## 8. Reproduce the numerical evidence

From the repository root:

```bash
python scrips/analyze_heading.py
```

The helper uses the Python standard library, reads the earlier and later archived CSV files, and prints JSON containing RMSE, checkpoint headings, accumulated turns, inferred separations, and observed percentage reductions. It does not run Gazebo or modify any data or configuration.

The equations are implemented in `src/navigation.py` and `src/localization/odometry_uncertainty.py`; timestamp matching is implemented in `src/localization/localization_metrics.py`. The source record for this diagnosis stays fixed at the archived earlier CSV above. Newer runs must not be substituted for the data used to discover this discrepancy.
