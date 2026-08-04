## Issue linkage

Closes #

Parent PRD: #

Related or blocked Issues:

## Outcome

Describe the observable behavior or repository capability delivered by this PR.

## Scope

- Included:
- Deliberately excluded:

## Acceptance

Map every Issue acceptance criterion to changed files and evidence.

- [ ] AC-001 — ...

## Rollback

Describe the smallest safe revert and any evidence that must remain available.

## Interface impact

- [ ] No Stable Interface change.
- [ ] Stable Interface impact is documented in the Issue and affected docs.
- [ ] Any changed names, types, QoS, parameters, units, ordering, errors, or cancellation behavior are covered by tests.
- [ ] An ADR is linked when the change is a qualifying architectural trade-off.

## Risks

List residual safety, compatibility, data, privacy, dependency, and operational risks.

## Dependencies

List prerequisite Issues, external constraints, approvals, or state `None`.

## Verification

List exact commands and results. Run the repository's full gate once on the final PR HEAD.

```text
python3 -m unittest tests.test_repository_contract
python3 scripts/check_repository.py --root .
bash scripts/verify.sh
```

Additional bounded evidence:

## Documentation and repository hygiene

- [ ] User-visible behavior and current status documentation are accurate.
- [ ] No build output, credentials, private audio/maps/bags, runtime evidence, or model weights are included.
- [ ] I reviewed the complete diff and recorded verification evidence.
