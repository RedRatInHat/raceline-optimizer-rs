# Security policy

## Supported versions

Until the first stable release, security fixes are applied to the latest commit
on the default branch and the latest published `0.1.x` release when practical.

## Reporting a vulnerability

Do not open a public issue for a vulnerability that could expose data, execute
untrusted code, load an unintended native library, or bypass input validation.
Report it privately to `alexpacuk@redratinhat.com` with:

- the affected version or commit;
- reproduction steps or a minimal input;
- expected and observed behavior;
- likely impact;
- any suggested mitigation.

You should receive an acknowledgement within seven days. Please allow time to
confirm the issue and prepare a coordinated fix before public disclosure.

## Scope notes

Track and vehicle JSON should be treated as untrusted input by applications that
expose the optimizer as a service. IPOPT is a native dependency; only load builds
from trusted sources and avoid accepting arbitrary library paths from remote
users. This repository does not contain the RaceLineCalc mobile application,
billing, advertising, or analytics services.
