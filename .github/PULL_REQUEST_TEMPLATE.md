## Work item

Closes #

Repository record: `docs/work-items/...`

## Outcome

Describe the observable behavior or learning result delivered by this change.

## Scope

- Included:
- Deliberately excluded:

## Verification

List exact commands and results. The full gate is required before merge.

```text
python3 -m unittest discover --start-directory tests --pattern 'test_*.py' --verbose
bash scripts/verify.sh
```

Additional bounded evidence:

## Interface and architecture

- [ ] ROS names, types, QoS, TF ownership, parameters, units, ordering, errors, and cancellation are unchanged or documented.
- [ ] Dependency directions in `docs/architecture/overview.md` remain true.
- [ ] LLM or speech output cannot bypass the strongly typed Mission and Motion Gate.
- [ ] A qualifying architectural trade-off has an ADR, or no ADR is needed.

## Safety and cleanup

- [ ] Motion is bounded and requests zero velocity in success and cleanup paths, or this change cannot command motion.
- [ ] Cancel, timeout, stale-result, and failure behavior is tested where relevant.
- [ ] Automated processes terminate and leave no Gazebo/ROS processes running.

## Documentation and repository hygiene

- [ ] User-visible behavior and current status documentation are accurate.
- [ ] `CHANGELOG.md`, lessons, and learning records are updated where applicable.
- [ ] No build output, credentials, private audio/maps/bags, runtime evidence, or model weights are included.
- [ ] I reviewed the complete diff and recorded verification evidence.
