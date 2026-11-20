# ASGI test worker-handoff diagnosis

Date: 2026-11-20. This diagnosis is specific to the available Ubuntu 24.04
WSL execution environment; it does not identify a general AnyIO release defect.

## Reproduction

The project environment uses Python 3.12.3, AnyIO 4.16.0, HTTPX 0.28.1,
Starlette 1.10.0, and FastAPI 0.162.7. The three request-level tests
`test_command_post_and_harness_views_are_separate`,
`test_api_validates_actions_and_reports_rejected_commands`, and
`test_scenario_api_validates_before_restart` blocked inside `asyncio.run()`.
The dashboard request stopped at `/harness`; the demo request stopped on a
handled 409 response. No assertion was removed or weakened.

A minimal `anyio.to_thread.run_sync(lambda: 3)` also failed to return. The
faulthandler snapshot showed the request coroutine awaiting the worker result,
the event loop idle in its selector, and an AnyIO worker idle in its queue.
The system Python has AnyIO 4.2.0; the same minimal call timed out there too.
The system environment has no HTTPX installation. A repeated standard-library
`asyncio.to_thread()` probe returned once and then blocked on the next worker
handoff. This evidence does not support attributing the fault to AnyIO 4.16.0
alone; it indicates worker/result delivery is unreliable in this runtime.

No compatible wheel was present in the inspected local caches. A bounded pip
download attempt failed because the package index hostname could not resolve.
No dependency, global configuration, or system package was changed.

## Correction and regression result

Command-post HTTP route functions are now async and keep their short local
callbacks on the ASGI loop. Dashboard HTML is loaded at app creation and served
as an HTML response. Static assets use a contained-path response handler rather
than Starlette `FileResponse`, avoiding the same worker-thread path. Existing
assertions remain unchanged; the dashboard test also fetches `app.js` and
fails if an AnyIO worker handoff is used.

Results after the correction: all three formerly blocked request tests passed;
the final full test suite passed **506/506** with zero skips, deselections, or
failures. The test client and app both stayed in the same asyncio event loop.

This correction avoids thread handoff in these short HTTP callbacks; it does
not claim the environment's general thread-pool behavior is repaired. A future
runtime with validated worker handoff can restore offloading for long-running
callbacks if needed.
