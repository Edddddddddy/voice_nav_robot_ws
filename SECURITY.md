# Security Policy

VoiceNav Robot is a simulation-only project. Its operational stop is not a
certified emergency-stop or functional-safety system, and no deployment on a
physical robot is supported.

## Supported versions

Only the latest released version receives security fixes. Development branches
and recovery archives are not supported releases.

## Reporting a vulnerability

Do not open a public issue for a suspected vulnerability, exposed credential,
or unsafe motion-bypass defect. Report it privately to `983166955@qq.com` with:

- the affected commit or release;
- a minimal reproduction;
- the expected and observed behavior;
- any known impact; and
- whether details have already been disclosed elsewhere.

Do not include real credentials, private audio, maps, bags, or model files in a
report. The maintainer will acknowledge receipt within seven days, assess
scope, and coordinate remediation and disclosure.

## Security boundaries

Local LLM output, ASR text, voice input, map IDs, and Named Place requests are
untrusted. They must not bypass Mission validation, admission fencing,
MotionGate, configured limits, or the consumer-side velocity timeout.

The project makes no safety claim for real hardware. Deploying this code on a
physical robot, exposing local model servers to a network, or treating
operational stop as a certified safety function is outside the supported scope.
