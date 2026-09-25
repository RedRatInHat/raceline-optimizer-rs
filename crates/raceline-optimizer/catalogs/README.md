# Shared stock vehicle catalogue

`vehicle-presets.v1.json` is the authoritative common catalogue for the public
Point, Car V1 and Moto V1 adapters. Consumers should vendor a specific commit
and exact byte digest rather than maintain independent numeric copies.
The resolver checks the embedded digest and version. Numeric changes require
a new catalogue version and reviewed reference fixtures.

`stock_snapshot.v1` means a complete reference profile, not a general
dependency-aware parameter editor. Stock references reject overrides; callers
needing a custom setup must supply an explicit full profile. No claim of
automatic component-inertia composition follows from selecting a stock ID.

Classes are generalized presets, not measured replicas of the vehicles named
by historical IDs. The reference equipped operator is 75 kg, already counted
once in the native total mass. Body dimensions are display/physical metadata;
optimization width is a separate native clearance parameter. These snapshots
are not a source of vehicle/operator component positions or intrinsic inertias.
Unmeasured calibration values, including inertias, remain estimates.

Follow parameter suffixes and the native contract for units: `_g` means
multiples of standard gravity, `_kw` means kilowatts, and `_w` means watts.
The historical `units: si` marker does not override these explicit suffixes.
Power aliases represent the same reference value in different units.

Run `cargo test -p raceline-optimizer --test stock_catalogue` to verify the
fourteen public profile maps, rejected invalid inputs and embedded digest.
