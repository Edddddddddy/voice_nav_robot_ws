# VN-NNNN: Short title

**Status:** Proposed

**GitHub Issue:**
[#NN](https://github.com/Edddddddddy/voice_nav_robot_ws/issues/NN)

**Delivery identity policy:** use `Refs #NN` while exact-final-head
verification, post-merge verification, or tag creation remains. Final local or
pushed HEAD, its gate result, public commit/tree, CI, and tag identities belong
in the PR and Issue closure comment because the target tree cannot contain its
own future identity.

## Goal

Describe one observable outcome.

## Non-goals

- State what this change deliberately does not include.

## Acceptance criteria

- [ ] Write measurable behavior or evidence.

## Risks and rollback

- Describe safety, compatibility, data, dependency, and operational risks.
- Describe how the change can be disabled or reverted.

## Design impact

- Stable Interfaces changed:
- TF or motion ownership changed:
- ADR required:

## Test plan

- Unit:
- Contract:
- Integration:
- Manual:

## Documentation

- Files that must change:

## Verification evidence

Record commands, exit status, and concise results after implementation. Link
large or private runtime artifacts instead of committing them.

Record only identities that already exist. The tree-level Work Item may become
`Done` when repository acceptance is complete while its linked Issue remains
open as the post-merge delivery ledger. After the final commit, run the gate on
that exact local/pushed HEAD and record its identity and result externally;
never commit that result back into the tree it verifies. After merge, publish
any required immutable artifact from reviewed public `main`, append its exact
identity to the Issue closure comment, and then close the Issue. Do not create
a recursive ledger PR solely to copy that future identity back into the tagged
tree.
