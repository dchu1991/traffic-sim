Invoke the PM (Project Manager) agent to plan and orchestrate a feature or fix end-to-end.

The PM agent will:
1. Analyse the codebase and identify affected files
2. Write a scoped task breakdown
3. Delegate implementation to ENG
4. Run QA + TEST in parallel once ENG completes
5. Loop back to ENG if issues are found (max 2 iterations)
6. Return a final summary: files changed, config keys added, tests written, docs updated

**Usage:** `/pm <task description>`

**Examples:**
- `/pm Add a --seed CLI argument to main.py for reproducible simulation runs`
- `/pm Fix the off-ramp controller so it doesn't adjust offramp_prob when destination mode is enabled`
- `/pm Add per-lane car count to the HUD`

Use `/eng` instead for small, well-defined single-file changes that don't need full QA/TEST orchestration.
