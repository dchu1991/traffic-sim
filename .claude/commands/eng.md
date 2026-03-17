Invoke the ENG (Engineering) agent directly to implement a specific, scoped change.

The ENG agent will:
1. Read CLAUDE.md and relevant source files
2. Implement the change following project conventions (IDM gap formula, config pattern, MOBIL sign)
3. Run `uv run pytest tests/ -x -q` before finishing
4. Return: files changed, methods added/modified, config keys added, test result

**Usage:** `/eng <precise description of what to implement>`

**Examples:**
- `/eng Add a --seed CLI argument to main.py that calls random.seed() and np.random.seed()`
- `/eng Fix the zipper merge so gap_behind is skipped when avg lane speed < zipper_speed_kmh`
- `/eng Add a max_speed_mult config key to cap the pygame speed multiplier`

For larger features that need QA review and new tests, prefer `/pm` which runs the full team.
