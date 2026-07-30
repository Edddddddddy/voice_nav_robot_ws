# VoiceNav Robot documentation

This tree separates the approved v1.0 target from implementation evidence.
Target documents are normative design contracts; they do not claim that the
current source already implements the described behavior.

## Product

- [v1.0 product specification](product/v1.0-product-spec.md)
- [Product glossary](product/glossary.md)
- [Engineering resources](product/resources.md)

The versioned product specification is the detailed acceptance contract.

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

## History and work

- [Architecture decisions](adr/)
- [Work items](work-items/)

ADRs and completed Work Items are historical records. Do not rewrite them to
make an earlier decision look current; add a superseding ADR or a new Work Item
instead.

## Status vocabulary

- **Current** means verified behavior present in the repository.
- **Target v1.0** means approved behavior that may still require implementation.
- **Evidence** means a command, result, or reviewed artifact tied to a Work Item.
