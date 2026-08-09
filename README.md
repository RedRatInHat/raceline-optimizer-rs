# RaceLine Optimizer

`raceline-optimizer` is a native Rust library and CLI for **racing-line
optimization**, **minimum-time trajectory optimization**, and reproducible track
geometry preparation. It computes locally optimized racing-line candidates by
minimizing modeled traversal time inside explicit left/right track boundaries
under configurable geometry and vehicle constraints.

The same solver core powers **RaceLineCalc**, where a track can be built from an
image and calculated without assembling the JSON pipeline manually.

- [RaceLineCalc website](https://redratinhat.com/products/racelinecalc/)
- [RaceLineCalc on Google Play](https://play.google.com/store/apps/details?id=com.racelinecalc.mobile)
- [RedRatInHat organization](https://github.com/RedRatInHat)

## Example racing-line calculation

![Clockwise double-track car minimum-time racing-line calculation with late apexes, speed extrema, braking and acceleration traces](https://raw.githubusercontent.com/RedRatInHat/raceline-optimizer-rs/master/docs/assets/raceline-car-clockwise.svg)

The overlay above is generated from a completed clockwise car solve on a
technical circuit. It shows the raw and normalized boundaries, calculation
stations, vehicle footprint checks, speed extrema, signed lateral acceleration,
and the transition from braking to drive. The optimized path repeatedly uses
the available track width and delays several apexes to improve corner exit.

## What it provides

The public workspace covers the complete numerical pipeline from metric track
boundaries to a trajectory that can be plotted, inspected, or consumed by
another application:

1. prepare solver stations from a closed circuit or open route;
2. validate the corridor topology and local section frames;
3. build a model-specific nonlinear program;
4. solve it through an external IPOPT runtime;
5. return a common trajectory, diagnostics, warnings, and visualization data.

### Track geometry and station generation

The input is an explicit metric corridor: paired left and right boundaries plus
route direction and, for open routes, start/finish geometry. Station generation
resamples that corridor into a deterministic solver representation containing a
centerline, paired boundaries, normals, section directions, and local widths.
Both exact and adaptive station counts are supported.

Before optimization, the geometry layer checks route topology, boundary
ordering, section-frame regularity, and corridor consistency. Prepared geometry
is hash-addressed so callers can reproduce the same request and detect stale or
mismatched station data.

### Point-mass racing-line optimizer

The point-mass model is the quickest way to calculate a physically constrained
racing-line candidate when detailed chassis and tire data are unavailable. It
optimizes lateral position inside the corridor, planar velocity, acceleration,
and segment time while preserving kinematic continuity.

Its speed-indexed acceleration envelope can use different drive, braking,
left-cornering, and right-cornering limits, coupled by a configurable exponent.
The objective minimizes modeled traversal time with smoothing terms for
acceleration, path offset, and velocity. This makes it useful for rapid track
studies, generic vehicle envelopes, and a stable initialization or comparison
case for the higher-detail models.

### Double-track car minimum-time model

The car solver is a space-domain, double-track nonlinear program with four tire
contact patches. A profile can describe mass, wheelbase, front/rear track width,
center-of-gravity position and height, yaw inertia, steering limits and response,
drive/brake response, axle force distribution, power, aerodynamic drag and
vertical load, rolling resistance, and front/rear tire behavior.

During optimization it evaluates four wheel loads and tire forces, longitudinal
and lateral load transfer, load-sensitive tire capacity, combined tire use,
power and braking limits, and control-rate constraints. The objective minimizes
modeled lap time for a closed circuit or traversal time for an open route while
regularization keeps the state and control trajectory numerically usable. This
model is intended for configurable road-car, race-car, and formula-style
profiles; it is an engineering optimizer, not a guarantee of measured vehicle
performance.

### Motorcycle minimum-time model

The motorcycle solver uses a lean-aware single-track model with front and rear
tire forces, load transfer, drag, rolling resistance, yaw and roll dynamics,
steering response, and directional drive/brake limits. It constrains normal
loads, combined tire use, slip angles, lean-related lateral acceleration, power,
and control rates while optimizing the path and speed profile together.

Motorcycle profiles can represent different mass, geometry, power, grip, lean,
and steering characteristics. The model is useful for comparative trajectory
studies and numerical experimentation; it should not be presented as a
validated high-fidelity motorcycle simulator or a safety-critical prediction.

### Kart racing-line profiles

Kart calculation is deliberately implemented as a kart-specific parameterization
of the double-track car solver rather than a fourth independent dynamics model.
Kart profiles supply the appropriate mass, compact geometry, footprint, power,
braking, drivetrain, and tire parameters while reusing the car minimum-time NLP
and its four-wheel constraints. This keeps kart results on the same contracts
and diagnostic path as the other vehicle families.

### Solver runtime, outputs, and quality diagnostics

The nonlinear programs are solved through a dynamically loaded IPOPT library;
IPOPT itself is not bundled. All public entry points use versioned JSON contracts
and typed error codes. Long-running solves expose progress and cooperative
cancellation, while the common result contains route position, XY trajectory,
heading, curvature, speed, longitudinal and lateral acceleration, tire or
envelope utilization, modeled time, normalized track geometry, warnings, and
visualization markers such as braking points and speed extrema.

The unified quality report combines residual, geometry, and smoothness metrics
available for the selected model. A clean hard gate means that none of the
supplied metrics crossed its configured threshold; it is not proof of global
optimality, physical validity, or safety. IPOPT solves a nonlinear local
optimization problem, so convergence and the returned candidate depend on the
track, profile, initialization, scaling, and native solver configuration.

The project can therefore serve as a racing line calculator, a minimum-lap-time
solver, and a Rust foundation for motorsport trajectory, path, and circuit
optimization without requiring the RaceLineCalc mobile application.

### More solved examples

All examples below use the same technical circuit so that the path and speed
differences come from the model/profile rather than from a different drawing.
Open an image to inspect the full-resolution station, acceleration, and speed
annotations. They illustrate particular configurations rather than a benchmark
or a claim that one vehicle family is inherently faster than another.

| Point-mass acceleration envelope | Lean-aware motorcycle |
| :---: | :---: |
| [![Point-mass racing-line optimization tuned for wide entries, late apexes, and full-width exits](https://raw.githubusercontent.com/RedRatInHat/raceline-optimizer-rs/master/docs/assets/raceline-point-mass-clockwise.svg)](https://raw.githubusercontent.com/RedRatInHat/raceline-optimizer-rs/master/docs/assets/raceline-point-mass-clockwise.svg) | [![Motorcycle minimum-time racing-line optimization with lean-aware dynamics](https://raw.githubusercontent.com/RedRatInHat/raceline-optimizer-rs/master/docs/assets/raceline-motorcycle-clockwise.svg)](https://raw.githubusercontent.com/RedRatInHat/raceline-optimizer-rs/master/docs/assets/raceline-motorcycle-clockwise.svg) |
| Illustrative wide-line tune: lower longitudinal acceleration authority and higher lateral acceleration limits make using the full corridor more valuable. | Single-track solve with front/rear tire and lean constraints. |

| Double-track car | Kart profile on the car solver |
| :---: | :---: |
| [![Double-track car minimum-time racing line with braking, drive, and speed annotations](https://raw.githubusercontent.com/RedRatInHat/raceline-optimizer-rs/master/docs/assets/raceline-car-clockwise.svg)](https://raw.githubusercontent.com/RedRatInHat/raceline-optimizer-rs/master/docs/assets/raceline-car-clockwise.svg) | [![Kart racing-line optimization using a kart-specific double-track vehicle profile](https://raw.githubusercontent.com/RedRatInHat/raceline-optimizer-rs/master/docs/assets/raceline-kart.svg)](https://raw.githubusercontent.com/RedRatInHat/raceline-optimizer-rs/master/docs/assets/raceline-kart.svg) |
| Four-wheel vehicle dynamics, load transfer, tire capacity, and power limits. | Compact kart parameters through the same minimum-time NLP and diagnostics. |

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
