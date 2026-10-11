# Correcting the simulated wheel contact geometry

## Diagnosis and implemented response

The earlier recorded run showed an approximately 6.05-degree clean-to-truth heading discrepancy after a nominal 45-degree turn. Reverse calculation gave an effective separation of **0.08815039 m**, close to the **0.088 m** inner-edge distance of the original 12 mm collision bands. The [heading diagnosis article](Differential_Drive_Heading_Error_Diagnosis_and_Effective_Wheel_Separation_EN.md) provides the complete derivation.

The selected model change was to reduce the collision width of **both** drive wheels from **0.012 m to 0.001 m**, keeping the 0.025 m radius and 0.10 m wheel-centre separation. The current model already contains this change.

## 1. Separate the relevant dimensions

| Quantity | Original collision model | Current collision model |
|---|---:|---:|
| Wheel centre-to-centre separation | 0.100 m | 0.100 m |
| Wheel radius | 0.025 m | 0.025 m |
| Collision cylinder width | 0.012 m | 0.001 m |
| Visual tyre width | 0.012 m | 0.012 m |
| Geometric inner-edge distance | 0.088 m | 0.099 m |

The physical wheel separation is defined by the wheel centres, at y = +0.05 m and -0.05 m. The inferred effective separation represents the observed turning response under an ideal-model assumption. Collision width controls the simulated contact band. These quantities must not be substituted for each other.

Changing the YAML separation to 0.0881 m would alter the planner and estimator equations. It could compensate for one observed scale mismatch without addressing the model's contact geometry and would require independent calibration tests. For the implemented correction, keep the nominal separation at 0.10 m and narrow the collision band instead.

## 2. Exact model modification

Edit `Round_Robot/models/round_robot/model.sdf`. Under each of `left_wheel` and `right_wheel`, locate `collision name="tire"`. Change only the cylinder length in its collision geometry.

Before:

```xml
<collision name="tire">
  <pose>0 0 0 1.57079632679 0 0</pose>
  <geometry>
    <cylinder>
      <radius>0.025</radius>
      <length>0.012</length>
    </cylinder>
  </geometry>
  <!-- Existing surface/friction settings remain here. -->
</collision>
```

After:

```xml
<collision name="tire">
  <pose>0 0 0 1.57079632679 0 0</pose>
  <geometry>
    <cylinder>
      <radius>0.025</radius>
      <length>0.001</length>
    </cylinder>
  </geometry>
  <!-- Existing surface/friction settings remain here. -->
</collision>
```

These excerpts illustrate the changed geometry; retain the existing complete collision element and its friction settings. The pose rotates the cylinder axis into the wheel axle direction, so cylinder length is its axial width. Apply the same width to both drive-wheel collisions. Do not change mesh scale, wheel visual width, radius, link poses, inertial values, or joint definitions as part of this correction.

Keep `config/robot.yaml` active values:

```yaml
robot_controller:
  ros__parameters:
    left_wheel_radius: 0.025
    right_wheel_radius: 0.025
    wheel_separation: 0.10
```

Keep the DiffDrive plugin at `<wheel_radius>0.025</wheel_radius>` and `<wheel_separation>0.10</wheel_separation>`. The commented 0.0881 m YAML candidate stays inactive.

The current repository already has two 0.001 m wheel collision widths. This procedure documents the existing change; it is not a request to apply another geometry alteration.

## 3. Why a narrower central band was chosen

With a finite-width wheel contact surface, physical turning can differ from ideal centreline rolling. Narrowing a centred collision band reduces the lateral range in which contact can occur, bringing the contact representation closer to a wheel-centreline approximation while leaving the visible tyre intact.

The original geometry's inner-edge distance matched the inferred turning scale closely. This made collision width a plausible parameter to investigate. The specific contact mechanism remains a hypothesis because the recorded data contains poses and wheel estimates, not contact locations, forces, or slip measurements.

## 4. Recorded comparison after the change

The earlier evidence is [localization_20261009_230014_130403.csv](../results/old/localization_20261009_230014_130403.csv); the later evidence is [localization_20261010_102534_404782.csv](../results/old/localization_20261010_102534_404782.csv). These are archived single-goal records associated with the before/after investigation.

| Metric | Earlier record | Later record |
|---|---:|---:|
| Matched samples | 1,457 | 729 |
| Final clean heading | 45.040216 degrees | 45.049836 degrees |
| Final truth heading | 51.094745 degrees | 45.481742 degrees |
| Final truth minus clean heading | 6.054529 degrees | 0.431906 degrees |
| Inferred effective separation | 0.08815039 m | 0.09905037 m |
| Clean position RMSE | 0.082983 m | 0.005988 m |
| Noisy position RMSE | 0.086259 m | 0.010525 m |
| Final clean position error | 0.149262 m | 0.010685 m |
| Final noisy position error | 0.164427 m | 0.026010 m |

The comparison calculations use all matched rows for RMSE and the last row for final errors:

```text
clean_RMSE = sqrt(sum(clean_error_m[i]^2) / N)
noisy_RMSE = sqrt(sum(position_error_m[i]^2) / N)
final_heading_difference = degrees(wrap(theta_gt_rad[last] - theta_clean_rad[last]))
observed_reduction_percent = 100 * (1 - later_value / earlier_value)

heading reduction = 100 * (1 - 0.4319062405 / 6.0545289022) = 92.866394%
clean RMSE reduction = 100 * (1 - 0.0059875581 / 0.0829833867) = 92.784630%
noisy RMSE reduction = 100 * (1 - 0.0105247945 / 0.0862588316) = 87.798589%
final noisy error reduction = 100 * (1 - 0.0260101759 / 0.1644273765) = 84.181359%
```

Use full CSV precision for calculation and round only the displayed results.

The observed final heading discrepancy is **92.87% lower**, clean position RMSE **92.78% lower**, noisy position RMSE **87.80% lower**, and final noisy position error **84.18% lower** in the later record.

These are reductions between stored runs. Different sample counts and the lack of complete configuration snapshots prevent treating them as a controlled estimate of collision width's isolated causal effect. No fresh simulation was executed for this article.

## 5. Inspect the turning checkpoints again

| Nominal clean checkpoint (degrees) | Time (s) | Actual clean heading (degrees) | Truth heading (degrees) | Truth minus clean (degrees) |
|---|---:|---:|---:|---:|
| 0 | 14.00 | 0.000000 | -0.000000 | -0.000000 |
| 15 | 14.60 | 14.810959 | 14.937917 | 0.126958 |
| 30 | 15.14 | 30.280819 | 30.563815 | 0.282996 |
| 45 | 15.88 | 44.971821 | 45.402949 | 0.431128 |

The later error still grows during the turn but is much smaller. At the sample nearest a clean 45-degree heading, the discrepancy is **0.431128 degrees**; after the turn completes, the final discrepancy is **0.431906 degrees**. Those values refer to different samples.

The later run's effective separation is approximately **0.09905037 m**, close to the **0.099 m** inner-edge distance of the 1 mm collision bands. A simplified inner-edge model predicts that a clean 45-degree turn would have excess true rotation of:

```text
45 * (0.100 / 0.099 - 1) = 0.454545 degrees
```

Its proximity to the recorded approximately 0.432-degree discrepancy is consistent with the same geometry hypothesis, but does not directly measure the contact points.

The equal-distance heading prediction is **0.010661 m**, compared with the recorded final clean position error of **0.010685 m**. Both the earlier and later records therefore show strong numerical consistency between their heading differences and clean endpoint errors.

## 6. Conditions for a comparable follow-up

The diagnosis above uses the recorded single-goal experiment toward `(1, 1)`. To test collision width independently, use that same goal and initial planar pose `(0, 0, 0)` for both model variants. Keep wheel radii, centre spacing, speed limits, noise settings, and feedback rates matched, and save the exact configuration with each new record.

Restart Gazebo after SDF changes and restart the estimator from the same known initial pose. Inspect clean heading differences and endpoint error. A wheel-target completion message alone does not prove accurate physical arrival.

Repeat matching clockwise and anticlockwise turns and compare multiple trials. Zero-noise trials and contact-point measurements would help isolate the proposed mechanism. These are follow-up methods, not additional evidence already collected for the historical diagnosis.

## 7. Recalculate the stored comparison

```bash
python scrips/analyze_heading.py
```

This standard-library helper prints the values used in both articles from the stored CSVs, including the observed reductions. It does not modify the model, generate new trials, or overwrite historical data.

The evidence for this investigation remains the two archived CSV files in Section 4. Keep these source records fixed when reproducing or citing this diagnosis; a newer experiment is a separate record and must not replace them.
