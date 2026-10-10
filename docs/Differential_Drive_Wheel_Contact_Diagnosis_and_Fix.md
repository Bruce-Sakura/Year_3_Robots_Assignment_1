# Differential-Drive Heading Error: Diagnosis and Collision-Geometry Fix

Date: 10 October 2026  
Environment: ROS 2 Jazzy and Gazebo Harmonic  
Repository: [Year_3_Robots_Assignment_1](https://github.com/Bruce-Sakura/Year_3_Robots_Assignment_1)

## 1. Problem

The robot's wheel-based odometry reported a turn of approximately 45 degrees, while its Gazebo ground-truth orientation showed approximately 51 degrees. The robot subsequently drove along the incorrect heading, producing a substantial position error.

The original experiment was recorded in `../results/old/localization_20261009_230014_130403.csv`. The heading difference was approximately constant during the following straight segment, indicating that the main systematic error accumulated during turning.

## 2. Model and Code Checks

The inspected model is `Round_Robot/models/round_robot/model.sdf`.

| Parameter | Original value |
|---|---:|
| Left wheel centre, y coordinate | +0.050 m |
| Right wheel centre, y coordinate | -0.050 m |
| Wheel centre-to-centre separation | 0.100 m |
| Wheel collision radius | 0.025 m |
| Wheel collision diameter | 0.050 m |
| Wheel collision cylinder width | 0.012 m |
| Gazebo DiffDrive wheel separation | 0.100 m |
| Gazebo DiffDrive wheel radius | 0.025 m |
| Drive-wheel friction coefficients, mu and mu2 | 1.0 |
| Ball-caster friction coefficients, mu and mu2 | 0.05 |

The wheel radius was correctly configured as 2.5 cm, corresponding to a 5 cm diameter. The geometric wheel separation was correctly configured as 10 cm between wheel centres.

The controller configuration in `config/robot.yaml` also used radii of `0.025` m and separation of `0.10` m. Inspection of `src/navigation.py` and `src/localization/odometry_uncertainty.py` found no radius-versus-diameter conversion error in the differential-drive calculations.

Gazebo's official Harmonic tutorial sets wheel separation from the difference between wheel-centre positions: wheels at y = +0.6 m and -0.6 m give a separation of 1.2 m. Therefore, wheel separation is not defined as the gap between the inner tyre edges.

Reference: [Gazebo Harmonic: Moving the robot](https://gazebosim.org/docs/harmonic/moving_robot/).

## 3. Mathematical Diagnosis

For ideal differential-drive odometry:

$$
\Delta s_L = r_L\Delta\phi_L,\qquad
\Delta s_R = r_R\Delta\phi_R
$$

$$
\Delta\theta_{\mathrm{odom}}
= \frac{\Delta s_R-\Delta s_L}{b_{\mathrm{configured}}}
$$

Here, wheel angles are in radians, wheel radii and separation are in metres, and heading change is in radians.

Assuming the wheel-distance conversion is accurate, an equivalent separation can be inferred from the observed heading change:

$$
b_{\mathrm{effective}}
=b_{\mathrm{configured}}
\frac{\Delta\theta_{\mathrm{odom}}}{\Delta\theta_{\mathrm{GT}}}
$$

The original final headings were:

| Measurement | Value |
|---|---:|
| Clean wheel-odometry heading | 45.040216 degrees |
| Ground-truth heading | 51.094745 degrees |
| Heading difference | 6.054529 degrees |

This gives:

$$
b_{\mathrm{effective}}
=0.100\frac{45.040216}{51.094745}
\approx 0.0881504\ \mathrm{m}
$$

A regression of ground-truth heading against clean heading during the steady turning portion produced a slope of approximately 1.13634764, giving:

$$
b_{\mathrm{effective,steady}}
=\frac{0.100}{1.13634764}
\approx 0.0880012\ \mathrm{m}
$$

The original collision width was 0.012 m. The distance between the two inner tyre edges was therefore:

$$
0.100-0.012=0.088\ \mathrm{m}
$$

This closely matches the inferred steady-turn separation. Meanwhile, the ground-truth straight displacement divided by the clean-odometry straight displacement was approximately 0.99999958 over the analysed straight segment. This strongly argues against a wheel-radius scale error as the cause of the much larger turning discrepancy.

**Interpretation:** the wide, flat cylindrical wheel collision geometry produced turning behaviour equivalent to a smaller effective separation. Contact concentrated near the inner tyre edges is a plausible mechanism supported by the numerical match. The exact contact-point distribution has not been directly measured, so this mechanism remains an inference rather than a verified contact trace.

The inferred 8.8 cm value is an effective kinematic parameter; it does not mean the model's wheel centres were physically only 8.8 cm apart.

## 4. Implemented Change

Both drive-wheel collision cylinders were narrowed from 12 mm to 1 mm. Only the collision geometry was changed; the wheel appearance, radius, wheel-centre positions, mass, inertia, friction settings, and controller parameters were retained.

Original collision geometry:

```xml
<geometry>
  <cylinder>
    <radius>0.025</radius>
    <length>0.012</length>
  </cylinder>
</geometry>
```

Updated collision geometry:

```xml
<geometry>
  <cylinder>
    <radius>0.025</radius>
    <length>0.001</length>
  </cylinder>
</geometry>
```

Apply this change to the collision cylinder under both `left_wheel` and `right_wheel`. The narrow cylinder remains centred on each wheel link, approximating a narrow central contact band while retaining the visible tyre width.

Keep the controller configuration unchanged:

```yaml
robot_controller:
  ros__parameters:
    left_wheel_radius: 0.025
    right_wheel_radius: 0.025
    wheel_separation: 0.10
```

The revised XML parsed successfully, and a content comparison confirmed that only the two collision widths changed. The updated model was supplied as a replacement file; the attempted GitHub write was rejected with HTTP 403, so that action did not update the repository.

## 5. Experimental Validation

After replacing the model and restarting the simulation, the new experiment was recorded in `../results/old/localization_20261010_102534_404782.csv`.

The plotting script reported:

| Approximate clean heading checkpoint | Ground-truth heading | Reported heading error |
|---|---:|---:|
| 0 degrees | -0.0000 degrees | -0.0000 degrees |
| 15 degrees | 14.9379 degrees | 0.1270 degrees |
| 30 degrees | 30.5638 degrees | 0.2830 degrees |
| 45 degrees | 45.4029 degrees | 0.4311 degrees |

The checkpoint labels are approximate. The script's error uses the actual selected clean heading, so it should not be recomputed by subtracting the rounded checkpoint label from the ground-truth value.

| Metric | Original 12 mm collision | Revised 1 mm collision |
|---|---:|---:|
| Heading error near 45 degrees | 6.0545 degrees | 0.4311 degrees |
| Noisy-estimate position RMSE | 0.086259 m | 0.010525 m |
| Final noisy-estimate position error | 0.164427 m | 0.026010 m |

Additional reported results for the revised run:

- Samples: 729
- Position MSE: 0.00011077 m²

The heading discrepancy decreased by approximately 92.9%. The observed position RMSE decreased by approximately 87.8%, and the final position error decreased by approximately 84.2%.

The heading comparison uses clean wheel odometry and ground truth, providing direct evidence of reduced systematic turning error. The position metrics include the artificially noisy estimate, so their improvement is an observed result of these runs rather than a noise-controlled measurement of collision geometry alone.

## 6. Remaining Error and Further Verification

The remaining heading discrepancy increases with the turn angle. For a 1 mm collision width, the inner-edge separation would be 0.099 m. If that separation governed turning, a clean 45-degree turn would produce an excess ground-truth rotation of:

$$
45\left(\frac{0.100}{0.099}-1\right)
\approx 0.4545\ \mathrm{degrees}
$$

This is close to the reported 0.4311-degree discrepancy and is consistent with the same contact-geometry explanation. It does not independently prove the exact contact location.

An optional next comparison is to reduce collision width to `0.0001` m (0.1 mm), restart the simulation, and repeat the same test. This has not yet been implemented or validated. Very narrow collision geometry should also be checked for contact stability before being adopted permanently.

For a stronger validation, repeat clockwise and anticlockwise turns at several angles, compare clean odometry with ground truth, and record contact locations if available. Assess noisy localisation separately, or disable injected noise for the model-validation run.

## 7. Conclusion

The original 10 cm wheel-centre separation and 5 cm wheel diameter were correctly configured. The experiment identifies wheel collision width as a major contributor to the systematic heading error: reducing the width from 12 mm to 1 mm reduced the near-45-degree heading discrepancy from approximately 6.05 degrees to 0.43 degrees.

The implemented remedy is a narrow central wheel collision band with the original visual geometry and centre-to-centre wheel separation retained. The proposed inner-edge contact mechanism is strongly supported by the effective-separation calculations and the width comparison, but direct contact measurements are still needed to establish the exact physical mechanism.
