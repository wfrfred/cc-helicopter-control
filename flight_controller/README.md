# Flight Controller Runtime

## Runtime Flow

The flight controller uses one active runtime flow:

```text
protocol input + sensor state
  -> mode lifecycle
  -> mode target
  -> controller
  -> mixer
  -> actuator protocol
```

Runtime code assumes ComputerCraft/CC:Sable globals such as `vector`, `matrix`,
`quaternion`, `sublevel`, `rednet`, `peripheral`, `parallel`, `os`, and `sleep`
exist. Flight-controller runtime modules must not import local API stubs or add
local vector/quaternion/matrix replacements.

## Mode Contract

Modes are state machines owned by `state/mode_state.lua`. The mode coordinator
selects the active mode and calls each mode with the same lifecycle contract:

```lua
mode:enter(ctx)
mode:update(ctx)
mode:exit(ctx)
mode:target(ctx)
mode:terms()
```

`update()` is the only lifecycle step that may advance mode state during the
normal control loop. `target()` is a pure snapshot read: repeated calls with the
same mode snapshot must not change mode, lock, cruise, navigation, or manual
integrator state.

The active mode target is controller-owned data. Telemetry and debug state come
from `mode_state:terms()`, not from controller targets.

## Locks And Modes

Height and heading locks are mode-internal release/capture state machines. They
are not a separate flight mode and are not a global target service.

`manual` and `position_hold` own axis locks because they have input release
semantics. While climb or heading input is active, the corresponding axis is
manual. After input release, the lock waits for the configured rate deadband or
relock timeout, then captures the current height or heading.

`cruise` does not own locks. On entry it captures three equivalent cruise
targets: horizontal velocity, height, and heading. Those targets remain frozen
until cruise exits.

`navigation` does not own locks. Its target overrides horizontal position,
height, and heading while navigation is active. When navigation exits, the next
mode decides its own axis state:

- lateral or heading manual override enters `manual`;
- climb-only override enters `position_hold`;
- inactive, complete, or cancel fallback enters `position_hold`.

Navigation exit relock is axis-specific. If the user is not commanding climb,
the receiving lock mode captures current height. If the user is not commanding
heading, it captures current heading. This relock does not decide the destination
mode.

## Cruise Semantics

`cruiseToggle` is one-shot input and only enters cruise from `manual`.

Cruise captures the current horizontal world velocity when entered. It also
captures current height and heading as cruise targets. The operator may still be
holding manual lateral input at the moment of entry; that held input must not
immediately exit cruise or force a release through `position_hold`.

After cruise is active, a new manual input edge, navigation activation, or other
explicit mode transition may exit cruise. Cruise has no lock release behavior of
its own.

## Sensor Age Policy

Sensor presence is required before control starts. After pose, velocity,
angular velocity, and navigation fields are present, stale sensor timestamps are
reported but do not force actuator commands to zero.

`config.control.sensor_age.warn_dt` and `fault_dt` define report-only thresholds.
When exceeded, `flight.reason` becomes `sensor_age_warning` or
`sensor_age_fault`, while `flight.name` remains `running`.

The controller prioritizes real-time operation over full sensor snapshot
synchronization. Input stale handling is separate: stale UI input is replaced by
the default zero input according to `config.control.input.stale_dt`.

## Runtime Boundaries

Adapters belong only at true external boundaries: protocol input, actuator
protocol, telemetry shaping, and UI rendering. Internal flight-controller code
must not add old-shape runtime paths.

Controller diagnostics live under `telemetry.control`. The top-level
`telemetry.command` is the final actuator command; duplicate final command
aliases should not be added.

## Verification

Before committing runtime changes, run:

```sh
sh tools/check_lua.sh
lua tools/smoke_test.lua
lua tools/run_control_fixture.lua
```

Run rejection scans for old-shape paths, invalid raw-state access,
debug-driven control input, local API stub imports, and old target fields.
