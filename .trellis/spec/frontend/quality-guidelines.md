# Quality Guidelines

> Frontend quality guidance for the current repo state.

---

## Current Reality

There is no standalone frontend application to lint, type-check, or component-test.

For UI-adjacent work in this repository:

- RViz config edits should be verified through the relevant launch flow
- RViz plugin package edits should be verified as catkin package changes

## Rule

- Do not claim browser-frontend validation when no browser frontend exists.
- Route most "UI" tasks back to backend/package validation.
- If a future frontend is introduced, define real lint, test, accessibility, and type-check rules at that time.
