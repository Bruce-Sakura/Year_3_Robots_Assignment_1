# Wheel collision geometry diagnosis

## Implemented model change

Repository history records a reduction in both drive-wheel collision cylinder widths from 0.012 m to 0.001 m. The current file is `Round_Robot/models/round_robot/model.sdf`. The wheel radii remain 0.025 m and the wheel centres remain at y = +0.05 m and -0.05 m. The visual tyre width remains 0.012 m.

The current collision geometry under both `left_wheel` and `right_wheel` is:

```xml
<cylinder>
  <radius>0.025</radius>
  <length>0.001</length>
</cylinder>
```

The active controller and DiffDrive separation remains 0.10 m. The width change is already present; no replacement model or external patch package is required.

## Archived experiment comparison

The table is recalculated from these stored CSV files:

- Earlier run: `results/old/localization_20261009_230014_130403.csv`.
- Later run: `results/old/localization_20261010_102534_404782.csv`.

These records occur before and after the collision-width change in project history. The CSV format does not embed the complete model, control, or noise configuration, so it cannot establish that collision width was the only experimental difference.

| Metric | Earlier run | Later run |
|---|---:|---:|
| Matched samples | 1,457 | 729 |
| Final ground-truth minus clean heading | 6.054529 degrees | 0.431906 degrees |
| Clean position RMSE | 0.082983 m | 0.005988 m |
| Noisy position RMSE | 0.086259 m | 0.010525 m |
| Final noisy localization error | 0.164427 m | 0.026010 m |

The later dataset has a smaller clean heading discrepancy and lower position RMSE. This is an observation from two stored runs, not a newly executed or controlled validation trial.

## Interpretation

A wide wheel collision band can affect contact behaviour during turning. For the original width, the geometric inner-edge distance is `0.100 - 0.012 = 0.088 m`, close to the equivalent separation inferred from the earlier heading data. This numerical agreement is a contact hypothesis; the CSV files do not contain contact-point measurements.

The model's mechanical wheel separation is centre to centre, not inner edge to inner edge. Keep its value at 0.10 m. A narrow central collision band changes simulated contact geometry while retaining visual tyre dimensions.

The current 0.001 m width and unchanged radius are documented in the root README and model notes. The assignment's three-waypoint results use `results/localization_20261010_105607_895741.csv`; do not substitute this archived single-goal comparison for the report's current dataset.
