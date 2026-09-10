# Upstream history and Rust transition

This repository remains a fork of
[`TUMFTM/global_racetrajectory_optimization`](https://github.com/TUMFTM/global_racetrajectory_optimization).
The fork relationship and upstream Git history are intentionally preserved.

## Historical Rust transition

- Last unchanged upstream commit: `a9995e2f5407f22eb7fb9dceac2b71a35276bb41`.
- The original Rust transition replaced the checked-out Python working tree
  with the independently developed RaceLineCalc Rust solver core.
- That original import came from RaceLineCalc commit
  `211647f5ef39f5f8b67cc12c24fc663ac988de1b`.
- `EXPORT-MANIFEST.json` preserves the original import's payload checksums. It
  is a historical provenance record, not a checksum manifest of the current
  0.2.0 files.

## Source-available snapshot

The 0.2.0 Rust snapshot is placed directly on the unchanged TUMFTM base above.
Its public Rust baseline was revision
`6b9dac8fa6e9c7d140ae1a27f19bc0066cf0ab79`, with reviewed general-purpose
cancellation, failure classification, CLI and progress corrections added.
Earlier Rust branches and tags are not republished by this snapshot. Their
absence does not revoke previously granted licenses or imply that historical
objects cannot remain accessible within GitHub's fork network.

No upstream commits were rewritten or removed from Git history. The Python
implementation, its sample datasets, and its images are absent from the current
working tree but remain available in commits up to the transition point.

## Implementation relationship

The Rust workspace originated in the solver core used by RaceLineCalc. It was
developed separately from the removed Python working tree and adds a reusable
Rust API, a standalone CLI, point-mass, car, and motorcycle models, station
generation, trajectory-quality validation, and native IPOPT integration.

The retained fork relationship documents historical context. It does not mean
that the current Rust files are generated translations of the removed Python
files.

## License boundaries

- The 0.2.0 distribution uses the source-available noncommercial license in
  `LICENSE`. The Licensor is PE Patsukevich Aleksandr (Red Rat in Hat), with
  commercial licensing requests at contact@redratinhat.com.

- Commits and files from the upstream history through
  `a9995e2f5407f22eb7fb9dceac2b71a35276bb41` remain governed by LGPL-3.0;
  the corresponding text is preserved as `LICENSE-LGPL-3.0`.
- The original Rust import and the 0.1.0 release were offered under
  `MIT OR Apache-2.0`. `LICENSE-MIT` and `LICENSE-APACHE` remain as historical
  notices. The new license does not revoke licenses already granted for those
  copies or relicense third-party material.

The historical transition references above describe provenance, not a
requirement to republish old Rust branches or tags in a later fork snapshot.
RaceLineCalc remains on its separate private solver;
V1.5/V2/V2.1 source and calibration are not included here. The retained GitHub
fork relationship is attribution of context, not TUMFTM endorsement.

When inspecting or redistributing an older revision, use the license that
applies to that revision and its files.
