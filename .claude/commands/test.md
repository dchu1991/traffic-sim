Invoke the TEST agent to write tests for a feature or run the existing suite and report gaps.

The TEST agent will:
- Follow `make_car` / `make_sim` factory patterns from `tests/helpers.py`
- Write tests in the correct `tests/test_*.py` file, one class per feature area
- Run `uv run pytest tests/ -v` and report pass/fail
- Report any bugs found (with `file:line`) — does NOT fix source

**Usage:** `/test [optional: feature name or file]`

With no argument, runs the full suite and reports coverage gaps.

**Examples:**
- `/test` — run all 96+ tests, report gaps
- `/test destination mode` — write tests for destination-mode behaviour
- `/test src/traffic_sim/recorder.py` — write tests for the recorder module
- `/test ramp controller` — write tests for dynamic ramp control logic
