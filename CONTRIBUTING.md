# Contributing

Contributions that improve correctness, numerical stability, documentation,
portable build behavior, and independently reproducible examples are welcome.

## Before changing code

1. Open an issue for broad model or public-contract changes.
2. Keep product-specific mobile, billing, analytics, and private fixture data out
   of this repository.
3. Use synthetic or clearly redistributable fixtures and document their origin.
4. Avoid changing JSON schema names or typed error codes without a migration
   note and tests.

## Local checks

```bash
cargo fmt --all -- --check
cargo clippy --workspace --all-targets -- -D warnings
cargo test --workspace
cargo package --list -p raceline-optimizer
cargo package --list -p raceline-optimizer-cli
```

Tests that require a locally installed IPOPT library are ignored by default.
New unit and contract tests should run without IPOPT whenever possible.

## Pull requests

- Keep each pull request focused.
- Explain numerical assumptions and units.
- Add regression coverage for behavior changes.
- Call out contract, convergence, or performance implications.
- Do not include generated solver outputs, native binaries, credentials, user
  tracks, or absolute local paths.

By contributing, you agree that your contribution may be distributed under the
current implementation's `MIT OR Apache-2.0` license choice. Historical upstream
material remains subject to its original LGPL-3.0 license.
