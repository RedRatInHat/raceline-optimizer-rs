# IPOPT setup

RaceLine Optimizer uses IPOPT for native nonlinear optimization. IPOPT is not
vendored or distributed by this repository; install a compatible build and make
its shared library discoverable at runtime.

## Library selection

The dynamic loader checks, in order:

1. The CLI `--ipopt-library PATH` value or the model-specific explicit path.
2. `RLC_IPOPT_LIBRARY`.
3. The platform default name:
   - Windows: `libipopt-3.dll`
   - macOS: `libipopt.dylib`
   - Linux and other Unix systems: `libipopt.so`

Example:

```bash
export RLC_IPOPT_LIBRARY=/opt/ipopt/lib/libipopt.so
cargo run -p raceline-optimizer-cli --bin raceline-optimize -- --help
```

PowerShell:

```powershell
$env:RLC_IPOPT_LIBRARY = "C:\path\to\libipopt-3.dll"
cargo run -p raceline-optimizer-cli --bin raceline-optimize -- --help
```

The IPOPT library and its dependent native libraries must target the same CPU
architecture as the Rust executable.

## Diagnosing failures

When the library cannot be loaded or a required native symbol is missing, the
solver returns a typed error:

```json
{
  "schema_version": "rust_solver_error.v1",
  "code": "solve.nativeBackendUnavailable",
  "error": "..."
}
```

The CLI writes this error to stderr, explains how to pass the library path, exits
non-zero, and does not create the requested trajectory output.

Check the following:

- The path points to the actual shared library, not only its directory.
- The Rust target and IPOPT build have matching architectures and ABIs.
- IPOPT's transitive native dependencies are on the loader search path.
- The chosen linear solver is included in the IPOPT build.

## Static-link feature

The `ipopt-static-link` crate feature switches the FFI boundary to linked IPOPT
symbols. It is intended for integrators that already provide the required native
link directives and libraries. The default CLI distribution uses dynamic
loading because it is easier to audit and configure across platforms.
