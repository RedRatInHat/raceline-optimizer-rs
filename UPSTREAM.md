# Upstream history and Rust transition

This repository remains a fork of
[`TUMFTM/global_racetrajectory_optimization`](https://github.com/TUMFTM/global_racetrajectory_optimization).
The fork relationship and upstream Git history are intentionally preserved.

## Transition point

- Last unchanged upstream commit: `a9995e2f5407f22eb7fb9dceac2b71a35276bb41`.
- The next commit, which introduces this file, replaces the checked-out Python
  working tree with the independently developed RaceLineCalc Rust solver core.
- The imported Rust snapshot comes from RaceLineCalc commit
  `211647f5ef39f5f8b67cc12c24fc663ac988de1b`.
- `EXPORT-MANIFEST.json` records every imported payload file together with its
  byte size and SHA-256 digest so the transition can be reproduced and audited.

No upstream commits were rewritten or removed from Git history. The Python
implementation, its sample datasets, and its images are absent from the current
working tree but remain available in commits up to the transition point.

## Implementation relationship

The current Rust workspace is the solver core used by RaceLineCalc. It was
developed separately from the removed Python working tree and adds a reusable
Rust API, a standalone CLI, point-mass, car, and motorcycle models, station
generation, trajectory-quality validation, and native IPOPT integration.

The retained fork relationship documents historical context. It does not mean
that the current Rust files are generated translations of the removed Python
files.

## License boundaries

- Commits and files from the upstream history through
  `a9995e2f5407f22eb7fb9dceac2b71a35276bb41` remain governed by LGPL-3.0;
  the corresponding text is preserved as `LICENSE-LGPL-3.0`.
- The newly imported Rust implementation and its new documentation are offered
  under `MIT OR Apache-2.0`, as described by `LICENSE-MIT` and
  `LICENSE-APACHE`.

When inspecting or redistributing an older revision, use the license that
applies to that revision and its files.
