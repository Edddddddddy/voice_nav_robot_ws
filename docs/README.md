# VoiceNav Robot documentation

This tree separates verified current behavior from the approved v1.0 target.
Target documents are normative design contracts; they do not claim that the
current source already implements the described behavior.

## Product

- [v1.0 product specification](product/v1.0-product-spec.md)
- [Product glossary](product/glossary.md)
- [Engineering resources](product/resources.md)

## Architecture

- [Architecture overview](architecture/overview.md)
- [Mission Runtime Interface](architecture/mission-runtime-interface.md)
- [Safety and motion contract](architecture/safety-and-motion-contract.md)
- [TF and operating modes](architecture/tf-and-operating-modes.md)
- [Voice and Agent contract](architecture/voice-and-agent.md)

## Process

- [Change lifecycle](process/change-lifecycle.md)
- [Quality policy](process/quality-policy.md)
- [Testing strategy](process/testing-strategy.md)
- [Release policy and roadmap](process/release-policy.md)
- [Problem recurrence control](process/problem-learning.md)
- [Known pitfalls](process/known-pitfalls.md)

## Governance

- [Architecture decisions](adr/)
- [Agent control protocol](agents/README.md)

ADRs remain historical decision records. Do not rewrite a decision to make an
earlier choice appear current; add a superseding ADR when the decision changes.

## Status vocabulary

- **Current** means verified behavior present in the repository.
- **Target v1.0** means approved behavior that may still require implementation.
- **Evidence** means a command, result, or reviewed artifact tied to a GitHub
  Issue, pull request, CI run, or immutable Git object.
