# RaceLine Optimizer

`raceline-optimizer` is a Rust library and command-line interface for racing-line
and minimum-time trajectory optimization. It supports point-mass, car, and
motorcycle models and uses IPOPT for native nonlinear optimization.

This working tree replaces the historical Python implementation while keeping
the original fork history intact. See [UPSTREAM.md](UPSTREAM.md) for the exact
transition point, provenance, and licensing boundaries.

## Workspace

- `crates/raceline-optimizer` — reusable solver library.
- `crates/raceline-optimizer-cli` — standalone `raceline-optimize` CLI and
  reproducible JSON examples.

Run the public test suite with:

```bash
cargo test --workspace
```

CLI usage and example inputs are documented in
[`crates/raceline-optimizer-cli/README.md`](crates/raceline-optimizer-cli/README.md).

## RaceLineCalc

This optimizer is the solver core used by **RaceLineCalc**, an application for
building, calculating, and visualizing racing trajectories from track geometry.

- [RaceLineCalc website](https://redratinhat.com/products/racelinecalc/)
- [RaceLineCalc on Google Play](https://play.google.com/store/apps/details?id=com.racelinecalc.mobile)
- [RedRatInHat on GitHub](https://github.com/RedRatInHat)

## License

The current Rust implementation is available under either the MIT License or
the Apache License 2.0, at your option. Historical upstream commits and removed
upstream files remain under LGPL-3.0. See [LICENSE](LICENSE) and
[UPSTREAM.md](UPSTREAM.md) for scope details.
