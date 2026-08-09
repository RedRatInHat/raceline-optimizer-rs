# raceline-optimize

`raceline-optimize` is the command-line interface for the public
`raceline-optimizer` Rust library. It prepares station geometry from track
boundaries and solves a point-mass, car, or motorcycle racing line.

## Optimize

```powershell
cargo run -p raceline-optimizer-cli --bin raceline-optimize -- `
  optimize `
  --track crates/raceline-optimizer-cli/examples/compact-oval-track.json `
  --vehicle crates/raceline-optimizer-cli/examples/point-mass-vehicle.json `
  --output target/compact-oval-result.json `
  --stations 80 `
  --ipopt-library ./vendor/ipopt/libipopt-3.dll
```

The IPOPT path can instead be supplied through `RLC_IPOPT_LIBRARY` when the
library is discoverable by the optimizer runtime.

The track file uses `TrackAreaContractV1`. The vehicle file uses
`raceline_optimizer_vehicle.v1` and selects one of `point_mass`, `car`, or
`bike`. The CLI owns the internal prepared-station request and its hashes; users
do not need to construct product or mobile request envelopes.

## Inspect

```powershell
cargo run -q -p raceline-optimizer-cli --bin raceline-optimize -- `
  inspect target/compact-oval-result.json
```

`inspect` validates the trajectory column lengths and prints a compact JSON
summary with lap time, speed range, maximum combined utilization, warnings, and
the unified quality gate.
