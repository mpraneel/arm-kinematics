Seeded run: 2000 commands, seed 42, dt 0.02 s.

| Metric | Supervisor off | On, hold | On, clamp | On, plan |
|---|---|---|---|---|
| Joint limit violations | 176 | 0 | 0 | 0 |
| Velocity limit violations | 423 | 0 | 0 | 0 |
| Collisions | 418 | 0 | 0 | 0 |
| Targets reached | 1928 / 2000 | 1238 / 2000 | 1247 / 2000 | 1205 / 2000 |
| Clean targets reached | 1196 / 1196 | 921 / 1196 | 930 / 1196 | 897 / 1196 |
| Intervention rate | 0.0% | 41.5% | 41.5% | 44.0% |
| p50 supervisor latency | 0.00 us | 4.17 us | 4.58 us | 4.38 us |
| p99 supervisor latency | 0.00 us | 11.13 us | 13.59 us | 2087.46 us |

Checks:

| Check | Supervisor off | On, hold | On, clamp | On, plan |
|---|---|---|---|---|
| reachability | 0 fired / 0 rejected | 92 fired / 92 rejected | 92 fired / 92 rejected | 92 fired / 92 rejected |
| joint_limits | 0 fired / 0 rejected | 71 fired / 0 rejected | 71 fired / 0 rejected | 53 fired / 0 rejected |
| velocity_limits | 0 fired / 0 rejected | 719 fired / 0 rejected | 717 fired / 0 rejected | 779 fired / 0 rejected |
| singularity | 0 fired / 0 rejected | 55 fired / 55 rejected | 55 fired / 55 rejected | 55 fired / 55 rejected |
| collision | 0 fired / 0 rejected | 54 fired / 54 rejected | 54 fired / 54 rejected | 43 fired / 43 rejected |
