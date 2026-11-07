# Reference implementation of the 1.2.22 VNAV constraint fix

These files mirror the working tree at the released 1.2.22 state
(2026-11-07): altitude-constraint qualifiers (`FL157A` = at or above,
`6890B` = at or below), the at-or-above descent-gradient guard, and the
250 KT / 10000 ft arming window.

Use this snapshot to restore 1.2.22 after a rollback. See
[`../VNAV_CONSTRAINT_FIX_1.2.22.md`](../VNAV_CONSTRAINT_FIX_1.2.22.md)
for the analysis, code snippets, hashes and verification steps.

Do not build from this folder directly.
