# raceline-optimize

`raceline-optimize` is the command-line interface for the public
`raceline-optimizer` Rust library. It prepares station geometry from track
boundaries and solves a point-mass, car, or motorcycle racing line.

Version 0.2.1 is source-available under the packaged LICENSE. Noncommercial
use, modification and redistribution are allowed; commercial use requires
prior written permission from PE Patsukevich Aleksandr (Red Rat in Hat),
contact@redratinhat.com. Historical MIT/Apache notices are retained for prior
releases and are not alternative licensing for this release as a whole.

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

Public stock presets can be selected with exactly one `preset_ref` instead of a
full `profile`. The `stock_preset_ref.v1` path supports only `point_mass`,
`car_v1`, and `moto_v1`; it does not accept parameter or width overrides.
Car V1 requires `solve_options.car_model_version="v1"`. Moto V1 requires
`bike_model_version="v1_experimental"` and
`moto_v1_formulation_mode="t1n_preproduct_v1"`. See the three
`examples/stock-*-vehicle.json` files. Full profiles remain available for custom
vehicle definitions.

## Prepare and replay

The advanced `prepare` command writes a `prepared_station_geometry.v4` document
without invoking any vehicle optimizer:

```powershell
raceline-optimize prepare --track track.json --output prepared.json --stations 160
```

The advanced `solve` command replays a complete versioned solver request without
rebuilding its prepared station geometry:

```powershell
raceline-optimize solve --model car --request request.json --output result.json
```

`--model` accepts `point_mass`, `car`, or `bike`. A prepared geometry document is
not a complete solver request: the request must also contain the matching
vehicle profile, source identity, solve options, and supported request schema.
Use `optimize` for ordinary track-plus-vehicle input. Both solve commands accept
`--ipopt-library`; replay requests using this override must have a `solve_options`
object. Malformed requests fail without producing a new trajectory result.

## Inspect

```powershell
cargo run -q -p raceline-optimizer-cli --bin raceline-optimize -- `
  inspect target/compact-oval-result.json
```

`inspect` validates the trajectory column lengths and prints a compact JSON
summary with lap time, speed range, maximum combined utilization, warnings, and
the unified quality gate.
