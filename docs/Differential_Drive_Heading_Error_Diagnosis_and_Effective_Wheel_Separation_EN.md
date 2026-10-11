# Recorded wheel-odometry heading diagnosis

## Evidence and scope

This note analyses the archived dataset `results/old/localization_20261009_230014_130403.csv`. It records a single turn-then-drive experiment with an endpoint near `(1, 1)`. It is separate from the current three-waypoint experiment in the assignment report.

The values below were recalculated directly from this CSV. The current code uses 0.025 m wheel radii and 0.10 m wheel separation. The active configuration does not use the equivalent separation derived below.

## Recorded measurements

| Metric | Value |
|---|---:|
| Matched samples | 1,457 |
| Final clean heading | 45.040216 degrees |
| Final ground-truth heading | 51.094745 degrees |
| Final ground-truth minus clean heading | 6.054529 degrees |
| Clean position RMSE | 0.082983 m |
| Noisy position RMSE | 0.086259 m |
| Final noisy localization error | 0.164427 m |

The heading discrepancy is present even in clean wheel odometry, so injected measurement noise alone cannot explain it. This dataset does not independently identify the physical contact mechanism.

## Equivalent separation calculation

For the differential-drive relation `dtheta = (dr-dl)/b`, assume the same wheel-travel difference explains the clean and true accumulated turns. Under that assumption:

```text
b_effective = b_nominal * clean_turn / true_turn
            = 0.10 * 45.040216 / 51.094745
            = 0.08815039 m
```

The calculation assumes negligible initial heading and uses final headings as accumulated turns. The resulting value is an equivalent rotational calibration parameter, not a measurement of physical wheel spacing. Contact effects, slip, geometry assumptions, or timing differences may be absorbed into it.

The current model wheel centres remain at y = +0.05 m and -0.05 m, giving a physical separation of 0.10 m. The candidate 0.0881 m in `config/robot.yaml` is commented out. Do not replace the model spacing with this inferred value solely on the basis of this run.

## Related evidence

The checked-in wheel collision width is now 0.001 m. See [Wheel collision geometry diagnosis](Differential_Drive_Wheel_Contact_Diagnosis_and_Fix.md) for the recorded comparison associated with the width change. Current assignment setup and results are documented in the root README.
