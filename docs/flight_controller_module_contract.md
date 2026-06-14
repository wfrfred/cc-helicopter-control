# Flight Controller Module Contract

This document defines the intended module boundaries for `flight_controller/`. The goal is not to split every concept into a separate file. The goal is to make ownership explicit: who reads data, who owns flight modes, who generates targets, who runs PID control, who mixes rotor outputs, and who reports telemetry.

The current design direction is:

```text
input_task + data_task
        ↓
control_task   ← owns flight-mode state machine
        ↓
controller
        ↓
rotor
        ↓
actuator_controller
```

`control_task.lua` is allowed to be the flight-control state machine. It should not be reduced to a meaningless thin wrapper. However, it should not contain sensor conversion math, PID implementation details, rotor mixing math, or telemetry formatting details.

---

## Design Principles

### 1. `control_task.lua` owns flight modes

Flight modes and mode transitions belong in `control_task.lua`, because this is where input, current state, target ownership, manual override, position hold, height hold, yaw hold, and future navigation meet.

There should not be a separate `guidance.lua` unless `control_task.lua` becomes too large after the state machine is made explicit. For now, a separate guidance module would mostly move complexity into another file without improving the interface.

### 2. Primitive modules do not own global flight state

Modules such as `target_state.lua`, `rate_lock.lua`, and `position_hold.lua` are primitives. They may store local state needed for their own algorithm, but they do not decide the global flight mode.

Examples:

```text
rate_lock.lua       owns one rate-to-hold target primitive
navigation.lua      owns raw-position target construction and FRD error projection
position_hold.lua   owns one horizontal position-control primitive
target_state.lua    owns manual roll/pitch target primitive
control_task.lua    owns which primitive is active and why
```

### 3. `data_task.lua` is the only raw sensor adapter

Raw Minecraft / Sable coordinates must be interpreted in `data_task.lua`. Other modules should consume the canonical structured state.

`state.raw.*` is not a general control interface. It may be used by boundary modules that need raw coordinates, such as telemetry/UI and navigation. Low-level control primitives consume `state.body.*` or already-projected FRD errors.

`body_axis` is a calibration/install definition used by `data_task.lua` to convert raw sensor vectors to body FRD vectors. It is not a runtime flight mode parameter.

### 4. Controller sees body state, not raw state

`controller.lua` should not know raw xyz, raw quaternion, rednet, input stale rules, rotor phase, or telemetry fields. It should receive a target and body state, then return body commands.

### 5. Rotor sees body commands, not controller internals

`rotor.lua` should receive commands as a table, not positional arguments. This avoids the historical `collective, roll, yaw, pitch` ordering trap.

---

## Canonical Data Contracts

### `shared.input`

Produced by `input_task.lua`. Consumed by `control_task.lua`.

```lua
shared.input = {
    roll = 0.0,   -- pilot roll input, normalized
    pitch = 0.0,  -- pilot pitch input, normalized
    yaw = 0.0,    -- pilot yaw input, normalized
    climb = 0.0,  -- pilot vertical input, normalized
    seq = nil,
    time = 0.0,
}
```

`input_task.lua` does not decide flight targets. It only publishes pilot input.

### `shared.state`

Produced by `data_task.lua`. Consumed mainly by `control_task.lua`, then passed down to controller and telemetry builder.

Keep this structure minimal. Do not cache derived magnitudes such as `total`, `horizontal`, `groundSpeed`, or `lateralSpeed` unless a module has a repeated measured need for them.

```lua
shared.state = {
    raw = {
        position = { x = 0.0, y = 0.0, z = 0.0 },
        velocity = { x = 0.0, y = 0.0, z = 0.0 },
        angularVelocity = { x = 0.0, y = 0.0, z = 0.0 },
        orientation = nil,
    },

    body = {
        pose = {
            down = 0.0,
            roll = 0.0,
            pitch = 0.0,
            yaw = 0.0,
        },

        velocity = {
            forward = 0.0,
            right = 0.0,
            down = 0.0,
        },

        rates = {
            roll = 0.0,
            pitch = 0.0,
            yaw = 0.0,
        },
    },

    time = {
        pose = 0.0,
        velocity = 0.0,
        rates = 0.0,
    },
}
```

`data_task.lua` may use a local runtime body basis internally:

```lua
local bodyBasis = {
    forward = orientation * config.calibration.body_axis.forward,
    right = orientation * config.calibration.body_axis.right,
    down = orientation * config.calibration.body_axis.down,
}
```

This basis does not need to be exposed unless a future module has a clear need for it.

### `target`

Owned and assembled by `control_task.lua`. Passed to `controller.lua`.

```lua
target = {
    attitude = {
        roll = 0.0,
        pitch = 0.0,
        source = "manual_attitude",
    },

    vertical = {
        down = 0.0,
        rate = 0.0,
        source = "height_hold",
    },

    yaw = {
        angle = 0.0,
        rate = 0.0,
        source = "yaw_hold",
    },

    position = nil,
}
```

The exact fields can be evolved, but the ownership should stay fixed: `control_task.lua` assembles targets; `controller.lua` consumes them.

### `commands`

Produced by `controller.lua`. Consumed by `rotor.lua`.

```lua
commands = {
    collective = 0.0,
    roll = 0.0,
    pitch = 0.0,
    yaw = 0.0,
}
```

All command passing should use named fields. Avoid positional APIs such as:

```lua
mixer:set(collective, roll, yaw, pitch)
```

Use:

```lua
mixer:setCommands(commands)
```

---

## Module Contracts

## `startup.lua`

### Does

Starts the flight-controller tasks and performs runtime initialization.

### Depends on

```text
config
data_task
input_task
control_task
telemetry_task
```

### Provides

A running flight-controller process. It should not provide flight-control logic.

### Must not do

```text
sensor conversion
PID control
rotor mixing
flight mode decisions
telemetry formatting
```

---

## `data_task.lua`

### Does

Reads raw physical state and publishes canonical structured state.

It is the only module that interprets raw sensor coordinates. It applies `config.calibration.body_axis` and converts raw vectors into body FRD pose, velocity, and rates.

### Depends on

```text
sublevel.getLogicalPose()
sublevel.getLinearVelocity()
sublevel.getAngularVelocity()
config.calibration.body_axis
config.runtime.data
lib.mathx
```

### Provides

```lua
shared.state.raw.position
shared.state.raw.velocity
shared.state.raw.angularVelocity
shared.state.raw.orientation

shared.state.body.pose
shared.state.body.velocity
shared.state.body.rates

shared.state.time.pose
shared.state.time.velocity
shared.state.time.rates
```

### Internal-only concepts

```text
body basis / runtime body axes
raw-to-body projection helpers
pose/rate conversion math
```

### Must not do

```text
pilot input interpretation
flight mode decisions
yaw-only navigation projection
position hold
PID control
rotor mixing
telemetry formatting
```

### Notes

`body.velocity` should be true body FRD velocity:

```lua
body.velocity.forward = rawVelocity:dot(bodyBasis.forward)
body.velocity.right = rawVelocity:dot(bodyBasis.right)
body.velocity.down = rawVelocity:dot(bodyBasis.down)
```

Yaw-only horizontal projection is a navigation or position-hold concern, not a base sensor-state concern.

---

## `input_task.lua`

### Does

Receives pilot input packets and publishes a normalized input snapshot.

### Depends on

```text
rednet
protocol input messages
config runtime network settings
```

### Provides

```lua
shared.input = {
    roll,
    pitch,
    yaw,
    climb,
    seq,
    time,
}
```

### Must not do

```text
target generation
flight mode transitions
PID control
position hold
height hold
yaw hold
```

---

## `control_task.lua`

### Does

Owns the main flight-control loop and the flight-mode state machine.

This is the correct place to decide which target source is active on each axis. It is allowed to coordinate manual input, height hold, yaw hold, position hold, and future navigation.

### Depends on

```text
shared.input
shared.state
config.control
config.runtime.control

target_state.lua
rate_lock.lua
navigation.lua
position_hold.lua
controller.lua
rotor.lua
telemetry_builder.lua
```

### Provides

```lua
shared.target          -- optional/debug
shared.controlResult   -- controller result / debug terms
shared.commands        -- latest body commands
shared.telemetrySource -- data used by telemetry_builder, if needed
```

It also calls:

```lua
controller:update({ target = target, state = shared.state.body, dt = dt })
rotor:setCommands(commands)
rotor:update()
```

### Owns

```lua
flight = {
    mode = {
        lateral = "manual_attitude", -- or "position_hold", "navigate"
        vertical = "height_hold",    -- or "manual_climb", "landing"
        yaw = "yaw_hold",            -- or "manual_yaw", "auto_yaw"
    },

    target = {
        attitude = ...,
        vertical = ...,
        yaw = ...,
        position = ...,
    },
}
```

### State-machine responsibilities

```text
manual roll/pitch input active:
    lateral mode becomes manual attitude
    position hold target is reset or suspended

roll/pitch released and position hold enabled:
    lateral mode becomes position hold
    current position is captured if needed

climb input active:
    vertical mode becomes manual climb

climb input released:
    vertical mode becomes height hold
    current height/down target is captured if needed

yaw input active:
    yaw mode becomes manual yaw

yaw input released:
    yaw mode becomes yaw hold
    current yaw target is captured if needed

future navigation command active:
    lateral mode becomes navigate
    control_task owns target lifetime
    navigation projects target/current position into FRD error

future landing command active:
    vertical mode becomes landing
    landing owns vertical target
```

### Must not do

```text
raw sensor coordinate conversion
PID implementation details
rotor blade mixing math
redstone/PWM output
monitor/UI drawing
telemetry field formatting details
```

### Notes

`control_task.lua` should be a meaningful owner of flight state, not a meaningless wrapper. The important constraint is that its dependencies expose clean primitives and structured contracts.

---

## `target_state.lua`

### Does

Primitive for manual roll/pitch target management.

It converts pilot roll/pitch input into attitude targets, including slew limits and return-to-home behavior.

### Depends on

```text
config.control target rates and limits
current body pose, if needed
pilot roll/pitch input
dt
```

### Provides

```lua
attitudeTarget = {
    roll = ...,
    pitch = ...,
}
```

or updates a target object owned by `control_task.lua`.

### Must not do

```text
height hold
yaw hold
position hold
PID command output
rotor mixing
flight-mode ownership
```

---

## `rate_lock.lua`

### Does

Generic primitive for manual rate input followed by hold-target capture.

This module can be used for vertical height/down hold and yaw hold. It is not itself a flight-mode manager.

### Depends on

```text
current value
manual input
rate limits
hold configuration
dt
```

### Provides

A lock result for one scalar axis:

```lua
lockResult = {
    target = ...,
    rate = ...,
    active = ...,
    manual = ...,
    error = ...,
    debug = ...,
}
```

### Used by

```text
control_task.lua
```

### Must not do

```text
decide global yaw mode
decide global vertical mode
call controller
call rotor
```

---

## `navigation.lua`

### Does

Converts raw position targets into FRD navigation errors.

This is a boundary primitive for code that must touch raw world coordinates. It may capture a raw position target and project the target/current horizontal delta into the current body FRD heading.

### Depends on

```text
state.raw.position
state.body.pose.yaw or equivalent heading information
navigation target data
```

### Provides

```lua
navigationTarget = {
    x = ...,
    z = ...,
}

bodyPositionError = {
    forward = ...,
    right = ...,
}
```

### Must not do

```text
own global lateral mode
run position PID
run final attitude PID
run rotor mixing
format telemetry
```

### Notes

Raw `x/z` position should stop here. Position-control primitives receive the projected FRD error, not raw coordinates.

---

## `position_hold.lua`

### Does

Primitive for horizontal position hold.

It converts FRD horizontal position error and FRD body velocity into attitude targets or position-control terms when lateral mode is position hold.

It does not own captured raw position targets. `control_task.lua` owns target lifetime and mode transitions; `navigation.lua` projects raw position targets into FRD error vectors.

### Depends on

```text
bodyPositionError.forward
bodyPositionError.right
state.body.velocity.forward
state.body.velocity.right
position-hold config
dt
```

### Provides

```lua
positionHoldResult = {
    attitude = {
        roll = ...,
        pitch = ...,
    },
    error = {
        forward = ...,
        right = ...,
    },
    debug = ...,
}
```

### Must not do

```text
own global lateral mode
capture raw position targets
read state.raw.position
read state.body.pose.yaw for navigation projection
run final attitude PID
run rotor mixing
interpret raw sensor axes
```

### Notes

Yaw-only horizontal projection belongs in `navigation.lua`, not in `data_task.lua` or `position_hold.lua`.

---

## `controller.lua`

### Does

Converts target and body state into body commands.

This is the stabilization/control law module. It owns PID instances and control terms, not flight modes.

### Depends on

```text
target
state.body.pose
state.body.velocity
state.body.rates
config.control
lib.pid
lib.mathx
dt
```

### Provides

```lua
result = {
    commands = {
        collective = ...,
        roll = ...,
        pitch = ...,
        yaw = ...,
    },

    terms = {
        height = ...,
        roll = ...,
        pitch = ...,
        yaw = ...,
    },

    debug = ...,
}
```

### Preferred API

```lua
local result = controller:update({
    target = target,
    state = shared.state.body,
    dt = dt,
})
```

### Must not do

```text
read shared directly
read raw sensor data
read rednet input
own flight mode transitions
capture position targets
mix rotor blades
send telemetry
```

---

## `rotor.lua`

### Does

Converts body commands into upper/lower rotor blade commands and broadcasts them to actuator controllers.

### Depends on

```text
commands.collective
commands.roll
commands.pitch
commands.yaw
rotor bearing angle/phase API
config.calibration.rotor phase offsets
config rotor/blade mount information
rednet/protocol rotor command messages
```

### Provides

```lua
rotorOutput = {
    upper = { ...blade outputs... },
    lower = { ...blade outputs... },
    debug = ...,
}
```

and sends rotor command messages to actuator controllers.

### Preferred API

```lua
mixer:setCommands({
    collective = ...,
    roll = ...,
    pitch = ...,
    yaw = ...,
})

mixer:update()
```

### Must not do

```text
PID control
flight mode decisions
sensor conversion
input stale handling
height/yaw lock logic
```

### Notes

Avoid positional command APIs. `collective, roll, yaw, pitch` ordering is too easy to misuse.

---

## `telemetry_builder.lua`

### Does

Builds a telemetry table from structured snapshots.

### Depends on

```text
shared.input
shared.state
target
controller result
rotor output / commands
mode state from control_task
```

### Provides

```lua
telemetry = {
    state = ...,
    input = ...,
    target = ...,
    commands = ...,
    modes = ...,
    debug = ...,
}
```

### Must not do

```text
control decisions
PID update
sensor conversion
rotor mixing
rednet receive loop
```

---

## `telemetry_task.lua`

### Does

Periodically sends the latest telemetry packet.

### Depends on

```text
shared.telemetry
rednet
protocol telemetry messages
config telemetry period/channel
```

### Provides

Telemetry packets to the UI computer.

### Must not do

```text
build complicated telemetry contents
control logic
sensor conversion
PID control
```

---

## Recommended File Layout

```text
flight_controller/
├── startup.lua
├── config.lua
│
├── data_task.lua
├── input_task.lua
├── control_task.lua
├── telemetry_task.lua
│
├── target_state.lua
├── rate_lock.lua
├── navigation.lua
├── position_hold.lua
├── controller.lua
├── rotor.lua
├── telemetry_builder.lua
│
└── lib/
    ├── mathx.lua
    ├── pid.lua
    └── protocol.lua
```

No separate `guidance.lua` is required in the current design. The guidance/state-machine role belongs to `control_task.lua`.

---

## First Refactor Phase

The first phase should not change flight behavior. It should only clarify interfaces.

### Change

```text
1. data_task publishes shared.state.raw/body/time.
2. data_task removes sensor_axis and uses body_axis only.
3. data_task keeps only three-axis raw/body vectors; derived speeds are computed by consumers.
4. control_task reads shared.state instead of scattered shared.pose/shared.rollRate/shared.velocity fields.
5. control_task owns explicit flight mode state.
6. navigation projects raw position targets into FRD errors before position_hold consumes them.
7. controller:update() receives { target, state, dt }.
8. rotor command API changes from positional args to setCommands(commands).
9. telemetry_builder consumes structured input/state/target/result/mode data.
```

### Do not change yet

```text
PID parameters
rotor phase math
blade mount math
position-hold algorithm
navigation/autoland behavior
actuator-controller PWM behavior
UI layout
```

---

## Open Design Questions

These should be resolved before adding navigation or autoland.

```text
1. Should body.velocity be true tilted-body FRD velocity only? Current answer: yes.
2. Should yaw-only horizontal velocity be exposed by data_task? Current answer: no.
3. Should control_task own flight mode state? Current answer: yes.
4. Should rate_lock remain? Current answer: yes, as a primitive, not as mode owner.
5. Should position_hold own lateral mode or raw target lifetime? Current answer: no; control_task owns mode/target lifetime, navigation projects raw targets, position_hold consumes FRD errors.
6. Should rotor commands use named table fields? Current answer: yes.
7. Should raw.position be visible outside data_task? Current answer: yes, but only for navigation and telemetry/UI, not for controller or position_hold.
```
