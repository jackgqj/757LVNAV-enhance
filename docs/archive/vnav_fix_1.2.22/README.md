# Archived implementation of the 1.2.22 VNAV constraint fix

These files are a verbatim copy of the working tree **before** the 1.2.22
change was rolled back (2026-11-07). They are kept so the fix can be
re-applied later without re-deriving it.

Re-application: copy `src/*` over the plugin `src/` (merge, not replace, for
files that changed after this snapshot), copy `tests/*` over `tests/`, and
then set the version to 1.2.22. See
[`../VNAV_CONSTRAINT_FIX_1.2.22.md`](../VNAV_CONSTRAINT_FIX_1.2.22.md)
for the analysis, code snippets and verification steps.

Do not build from this folder directly.
