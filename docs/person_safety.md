# Person safety

A worker crosses the aisle on an event-triggered path. The fused person becomes
a Nav2 obstacle and the collision monitor owns the final velocity stop. A valid
run requires:

- a worker spawn event;
- a collision-monitor `STOP` state;
- commanded linear velocity below 0.02 m/s while the worker blocks the aisle;
- motion resuming after the worker clears the aisle;
- zero physical/proximity collision events.

The benchmark records `stop_events` and `person_stop_success`; the final gate
requires 100% success across all recorded runs.
