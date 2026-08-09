# Releasing

The library and CLI use the same version, but crates.io must receive them in
dependency order.

## Local release checks

From a clean working tree:

```bash
cargo fmt --all -- --check
cargo clippy --workspace --all-targets -- -D warnings
cargo test --workspace
cargo package -p raceline-optimizer
cargo package --list -p raceline-optimizer-cli
```

Confirm that package contents contain only public Rust sources, synthetic
fixtures, examples, documentation, and license files. Native libraries, solver
outputs, product fixtures, absolute paths, and credentials must not be present.

## Publish order

1. Publish `raceline-optimizer`.
2. Wait until crates.io serves that exact version from its index.
3. Package and publish `raceline-optimizer-cli`, whose dependency pins the same
   library version while retaining a local `path` for workspace development.
4. Create the matching Git tag and GitHub release only after both package pages
   and their documentation links resolve.

The CLI package cannot complete crates.io dependency verification before the
matching library version exists in the registry. That is expected Cargo behavior,
not a reason to remove the dependency version.

Publishing packages, pushing tags, and creating a GitHub release are explicit
maintainer actions; CI only validates the source tree.
