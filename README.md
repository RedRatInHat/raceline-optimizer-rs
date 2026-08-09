# RaceLine Optimizer

`raceline-optimizer` is a native Rust library and CLI for **racing-line
optimization**, **minimum-time trajectory optimization**, and reproducible track
geometry preparation. It turns left/right track boundaries into an optimal,
ideal, or fastest racing line for car, motorcycle, kart, and point-mass vehicle
profiles.

The same solver core powers **RaceLineCalc**, where a track can be built from an
image and calculated without assembling the JSON pipeline manually.

- [RaceLineCalc website](https://redratinhat.com/products/racelinecalc/)
- [RaceLineCalc on Google Play](https://play.google.com/store/apps/details?id=com.racelinecalc.mobile)
- [RedRatInHat organization](https://github.com/RedRatInHat)

## What it provides

- Boundary-to-station geometry generation for closed circuits and open routes.
- Point-mass racing-line calculation with acceleration-envelope constraints.
- Double-track car minimum-time optimization.
- Single-track motorcycle optimization with lean dynamics.
- Car-based kart profiles for kart racing-line optimization.
- IPOPT-backed nonlinear programming with deterministic JSON contracts.
- Progress, cancellation, trajectory diagnostics, and a unified quality gate.
- A standalone `raceline-optimize` CLI for JSON input/output automation.

The project is useful as a racing line calculator, a minimum-lap-time solver,
and a Rust foundation for motorsport trajectory, path, and circuit optimization.

## Workspace

| Package                  | Purpose                                                                                     |
| ------------------------ | ------------------------------------------------------------------------------------------- |
| `raceline-optimizer`     | Solver library, public contracts, station generation, vehicle dynamics, and quality checks. |
| `raceline-optimizer-cli` | `raceline-optimize optimize` and `raceline-optimize inspect`.                               |

Only the reusable optimizer and CLI are public here. RaceLineCalc mobile UI,
billing, analytics, image autotracing, FFI packaging, and private product
regression fixtures remain outside this repository.

## Requirements

- Rust 1.85 or newer.
- A compatible IPOPT shared library for actual solves.

The project dynamically loads IPOPT by default; native binaries are not bundled.
Pass the library with `--ipopt-library`, or set `RLC_IPOPT_LIBRARY`. See
[IPOPT setup](docs/IPOPT.md) for platform-specific names and diagnostics.

## Quick start

Clone the repository and verify the public workspace:

```bash
cargo test --workspace
```

Run the included point-mass example:

```bash
cargo run -p raceline-optimizer-cli --bin raceline-optimize -- \
  optimize \
  --track crates/raceline-optimizer-cli/examples/compact-oval-track.json \
  --vehicle crates/raceline-optimizer-cli/examples/point-mass-vehicle.json \
  --output target/compact-oval-result.json \
  --stations 80 \
  --ipopt-library /path/to/libipopt.so
```

Inspect the generated trajectory without solving it again:

```bash
cargo run -q -p raceline-optimizer-cli --bin raceline-optimize -- \
  inspect target/compact-oval-result.json
```

On Windows PowerShell, replace the trailing `\` line continuations with
backticks and pass a compatible `libipopt-3.dll` path. More examples, including
kart and motorcycle profiles, are in the
[CLI documentation](crates/raceline-optimizer-cli/README.md).

## Input and output contracts

The CLI intentionally keeps product-internal request envelopes out of the user
interface:

1. `--track` accepts a `TrackAreaContractV1` JSON document containing the left
   and right boundaries, units, route mode, direction, and optional open-route
   start/finish geometry.
2. `--vehicle` accepts `raceline_optimizer_vehicle.v1` and selects
   `point_mass`, `car`, or `bike` plus its vehicle profile.
3. The CLI generates deterministic station geometry and dispatches the matching
   public solver API.
4. Successful solves emit `rust_solver_response.v1`; failures emit the typed
   `rust_solver_error.v1` schema.

The examples are small synthetic tracks and profiles intended to be copied and
modified. See [architecture](docs/ARCHITECTURE.md) for the full data flow.

## Library API

The stable integration boundary starts in `raceline_optimizer::solver_api`:

- `solve_point_mass_json`
- `solve_car_mintime_json`
- `solve_bike_mintime_json`
- progress/cancellation variants for each solve family
- `build_station_geometry_json`

These functions preserve typed error codes and the same JSON output contracts as
the CLI. Lower-level modules remain public in `0.1.x` for research and advanced
integration, but may be narrowed before `1.0`.

## Known limits

- IPOPT availability and its native linear-solver configuration are external to
  this crate.
- Input boundaries must describe a coherent corridor; folded or crossing station
  sections are rejected by topology and section-frame validation.
- Numerical convergence is model-, track-, initialization-, and scale-dependent.
- The repository does not include a GUI or image-to-track extraction.
- The public API is pre-1.0 and may evolve with explicit release notes.

## Development

Before opening a pull request, run:

```bash
cargo fmt --all -- --check
cargo clippy --workspace --all-targets -- -D warnings
cargo test --workspace
```

See [CONTRIBUTING.md](CONTRIBUTING.md), [SECURITY.md](SECURITY.md), and the
[GitHub Actions workflow](.github/workflows/ci.yml). Maintainers should also use
the [release checklist](docs/RELEASING.md), which records the required library →
CLI publish order.

## Upstream history and license

This repository preserves its historical fork relationship with
`TUMFTM/global_racetrajectory_optimization`, while the current working tree is
an independently developed Rust implementation imported from RaceLineCalc.
Exact transition commits, checksums, and license boundaries are documented in
[UPSTREAM.md](UPSTREAM.md) and [EXPORT-MANIFEST.json](EXPORT-MANIFEST.json).

The current Rust implementation is available under either MIT or Apache-2.0 at
your option. Historical upstream commits and removed upstream files remain under
LGPL-3.0. See [LICENSE](LICENSE) for the concise scope statement.
