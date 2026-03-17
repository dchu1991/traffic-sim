Invoke the QA agent to review uncommitted changes or a specific file for correctness and documentation accuracy.

The QA agent will check:
- Gap formula (`gap = leader.position - car.position - leader.length`) unchanged
- Circular road arithmetic uses `% road_length`
- Config completeness: new params have dataclass defaults + TOML keys
- MOBIL criterion sign is correct
- Exiting-car invariants preserved
- `docs/` and `CLAUDE.md` are in sync with code changes

Returns: **PASS / NEEDS_WORK / FAIL** with `file:line` citations.

**Usage:** `/qa [optional: path or description]`

With no argument, reviews all files modified since last commit (`git diff HEAD`).

**Examples:**
- `/qa` — review everything in `git diff HEAD`
- `/qa src/traffic_sim/simulation.py` — review a specific file
- `/qa destination mode changes` — review recent destination-mode work
