# Security Policy

VoiceNav Robot is a simulation-only learning project. Its operational stop is
not a certified emergency-stop or functional-safety system.

## Supported versions

Only the latest released version receives security fixes. Development branches
and lesson start tags are teaching checkpoints and are not supported releases.

## Reporting a vulnerability

Do not open a public issue for a suspected vulnerability, exposed credential,
or unsafe motion-bypass defect. Report it privately to
`983166955@qq.com` with:

- the affected commit or release;
- a minimal reproduction;
- the expected and observed behavior;
- any known impact;
- whether details have already been disclosed elsewhere.

Do not include real credentials, private audio, maps, bags, or model files in
the report. The maintainer will acknowledge receipt within seven days, assess
scope, and coordinate remediation and disclosure.

## Security boundaries

The local LLM, ASR text, Voice input, map IDs, and Named Place requests are
untrusted inputs. They must not bypass Mission validation, admission fencing,
the Motion Gate, configured limits, or consumer-side velocity timeout.

The project makes no safety claim for real hardware. Deploying this code on a
physical robot is outside the supported scope.
