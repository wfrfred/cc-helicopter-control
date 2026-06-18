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

`control_task.lua` is allowed to be the flight-control state machine. It should not be reduced to a meaningless thin wrapper. However, it should not contain sensor conversion math, PID implementation details, rotor mixing math, or UI drawing details.

---

## Design Principles

### 1. `control_task.lua` owns flight modes

Flight modes and mode transitions belong in `control_task.lua`, because this is where input, current state, target ownership, manual override, position hold, height hold, heading hold, and future navigation meet.

There should not be a separate `guidance.lua` unless `control_task.lua` becomes too large after the state machine is made explicit. For now, a separate guidance module would mostly move complexity into another file without improving the interface.

### 2. Primitive modules do not own global flight state

Modules such as `target_state.lua`, `rate_lock.lua`, and `position_hold.lua` are primitives. They may store local state needed for their own algorithm, but they do not decide the global flight mode.

Examples:

```text
rate_lock.lua       owns one rate-to-hold target primitive
position_hold.lua   owns one horizontal position-control primitive
target_state.lua    owns manual roll/pitch target primitive
lib/mathx.lua       owns projection/component primitives
control_task.lua    owns which primitive is active and why
```

### 3. `data_task.lua` is the only raw sensor adapter

Raw Minecraft / Sable coordinates must be interpreted in `data_task.lua`. Other modules should consume the canonical structured state.

`state.raw.*` is not a general control interface. It may be used by boundary code that needs raw coordinates, such as telemetry/UI and `control_task.lua` target capture. Low-level control primitives consume `state.body.*` or already-projected FRD errors.

`body_axis` is a calibration/install definition used by `data_task.lua` to convert raw sensor vectors to body FRD vectors. It is not a runtime flight mode parameter.

### 4. Controller sees body state, not raw state

`controller.lua` should not know raw xyz, raw sensor quaternion, rednet, input stale rules, rotor phase, or telemetry fields. It should receive a target and body state, including body-frame attitude quaternions, then return body commands.

### 5. Rotor sees body commands, not controller internals

`rotor.lua` should receive commands as a table, not positional arguments. This avoids the historical `collective, roll, yaw, pitch` ordering trap.

---

## Canonical Data Contracts

### `shared.input`

Produced by `input_task.lua`. Consumed by `control_task.lua`.

```lua
shared.input = {
    controls = {
        roll = 0.0,   -- pilot roll input, normalized
        pitch = 0.0,  -- pilot pitch input, normalized
        heading = 0.0, -- pilot heading input, normalized
        climb = 0.0,  -- pilot vertical input, normalized
    },

    event = {
        cruiseLock = false, -- latched request to lock current horizontal velocity
    },
    seq = nil,
    time = 0.0,
}
```

`input_task.lua` does not decide flight targets. It only publishes pilot input.

### `shared.state`

Produced by `data_task.lua`. Consumed mainly by `control_task.lua`, then passed down to controller and telemetry snapshots.

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
        frame = {
            forward = { x = 0.0, y = 0.0, z = -1.0 },
            right = { x = 1.0, y = 0.0, z = 0.0 },
            down = { x = 0.0, y = -1.0, z = 0.0 },
        },

        orientation = {
            w = 1.0,
            x = 0.0,
            y = 0.0,
            z = 0.0,
        },

        pose = {
            height = 0.0,
            roll = 0.0,
            pitch = 0.0,
            heading = 0.0,
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

This basis is exposed as `state.body.frame` because controller attitude error is computed in the current body frame. `state.body.pose.pitch` follows the FRD right-hand convention: positive pitch is nose up, so `frame.forward.y > 0` produces positive pitch. `state.body.pose.heading` is the navigation heading from `atan2(forward.x, -forward.z)`, not a body yaw component.

### `target`

Owned and assembled by `control_task.lua`. Passed to `controller.lua`.

```lua
target = {
    attitude = {
        roll = 0.0,
        pitch = 0.0,
        source = "manual",
        orientation = ...,
    },

    vertical = {
        height = 0.0,
        speed = 0.0,
        active = true,
        pending = false,
        error = 0.0,
        source = "locked",
    },

    heading = {
        angle = 0.0,
        active = true,
        pending = false,
        error = 0.0,
        source = "locked",
    },

    position = nil,
}
```

The exact fields can be evolved, but the ownership should stay fixed:
`control_task.lua` assembles targets. `controller.lua` consumes only the
controller-facing subset: attitude orientation/metadata and vertical target.
`heading` is navigation/input telemetry and target-generation state; it is not a
controller input.

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

### `shared.telemetry`

Produced by `control_task.lua`. Sent by `telemetry_task.lua` and consumed by the UI.

Telemetry is a structured snapshot, not a flattened list of prefixed fields:

```lua
shared.telemetry = {
    status = "running",
    time = ...,
    dt = ...,

    age = {
        pose = ...,
        rates = ...,
        velocity = ...,
    },

    input = {
        controls = { roll = ..., pitch = ..., heading = ..., climb = ... },
        event = { cruiseLock = ... },
        age = ...,
        stale = ...,
        sender = ...,
    },

    mode = {
        lateral = "manual", -- or "cruise", "position_hold", "navigation"
        vertical = "height_hold",
        heading = "heading_hold",
    },

    lock = {
        height = "locked", -- or "manual", "pending"
        heading = "locked", -- or "manual", "pending"
    },

    state = {
        raw = ...,
        body = ...,
    },

    output = {
        commands = ...,
        collective = ...,
        attitude = {
            roll = ...,
            pitch = ...,
            yaw = ...,
        },
        rotor = ...,
    },

    pid = {
        vertical = ...,
        position = ...,
        velocity = ...,
        attitude = {
            roll = { angle = ..., rate = ... },
            pitch = { angle = ..., rate = ... },
            yaw = { angle = ..., rate = ... },
        },
    },

    target = {
        vertical = ...,
        attitude = {
            roll = { angle = ..., rate = ... },
            pitch = { angle = ..., rate = ... },
            yaw = { angle = ..., rate = ... },
        },
        heading = ...,
    },

    current = {
        vertical = ...,
        attitude = {
            roll = { angle = ..., rate = ... },
            pitch = { angle = ..., rate = ... },
            yaw = { angle = ..., rate = ... },
        },
        heading = ...,
    },

    error = {
        vertical = ...,
        attitude = {
            roll = { angle = ..., rate = ... },
            pitch = { angle = ..., rate = ... },
            yaw = { angle = ..., rate = ... },
        },
        heading = ...,
    },
}
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
lib.mathx
lib.attitude_math
```

### Provides

```lua
shared.state.raw.position
shared.state.raw.velocity
shared.state.raw.angularVelocity
shared.state.raw.orientation

shared.state.body.orientation
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
mathx.project axis declarations
pose/rate conversion math
```

### Must not do

```text
pilot input interpretation
flight mode decisions
heading-level horizontal control projection
position hold
PID control
rotor mixing
telemetry formatting
```

### Notes

`body.velocity` should be true body FRD velocity:

```lua
body.velocity = mathx.project(rawVelocity, {
    forward = bodyBasis.forward,
    right = bodyBasis.right,
    down = bodyBasis.down,
})
```

Heading-level horizontal projection is a target-selection concern in `control_task.lua`, not a base sensor-state concern.

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
    controls = {
        roll,
        pitch,
        heading,
        climb,
    },
    event = {
        cruiseLock,
    },
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
heading hold
```

---

## `control_task.lua`

### Does

Owns the main flight-control loop and the flight-mode state machine.

This is the correct place to decide which target source is active on each axis. It is allowed to coordinate manual input, height hold, heading hold, position hold, and future navigation.

### Depends on

```text
shared.input
shared.state
config.control
config.runtime.control

target_state.lua
rate_lock.lua
position_hold.lua
controller.lua
rotor.lua
lib.mathx
```

### Provides

```lua
shared.target          -- optional/debug
shared.controlResult   -- controller result / debug terms
shared.commands        -- latest body commands
shared.telemetry       -- structured telemetry snapshot
```

It also calls:

```lua
controller:update({ target = target, state = controlState, dt = dt })
rotor:setCommands(commands)
rotor:update()
```

### Owns

```lua
lateralMachine = {
    mode = "manual", -- or "cruise", "position_hold", "navigation"
    positionTarget = ...,
    cruiseVelocity = ...,
    navigationTarget = ...,
}

heightLock = rate_lock.new(...)
headingLock = {
    target = ...,
    pending = ...,
}

target = {
    attitude = ...,
    vertical = ...,
    heading = ...,
    position = ...,
}
```

Telemetry derives `mode` and `lock` directly from these owners. Do not keep a
separate mirror table that only copies fields back into telemetry.

### State-machine responsibilities

```text
manual roll/pitch input active:
    lateral mode becomes manual
    position hold target is reset or suspended

velocity cruise command active:
    lateral mode becomes cruise
    cruise velocity is tracked in navigation heading-level FRD

caps lock pressed:
    current horizontal velocity is captured in navigation heading-level FRD
    lateral mode becomes cruise
    if roll/pitch is already active, that held input is ignored until it returns to center

manual roll/pitch input while cruising:
    cruise velocity target is cleared
    lateral mode becomes manual

roll/pitch released and position hold enabled:
    lateral mode becomes position hold
    current position is captured if needed

climb input active:
    vertical mode stays height_hold
    height lock substate becomes manual

climb input released:
    vertical mode stays height_hold
    height lock substate becomes pending, then locked
    current absolute height target is captured if needed

heading input active:
    heading mode stays heading_hold
    heading lock substate becomes manual_lookahead
    A/D creates an instantaneous lookahead heading target from current heading

heading input released:
    heading mode stays heading_hold
    heading lock substate becomes pending, then locked
    current heading target is captured if needed

future navigation command active:
    lateral mode becomes navigation
    control_task owns target lifetime
    control_task projects target/current position into FRD error with mathx.project

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
```

### Notes

`control_task.lua` should be a meaningful owner of flight state, not a meaningless wrapper. The important constraint is that its dependencies expose clean primitives and structured contracts.

---

## `target_state.lua`

### Does

Primitive for manual roll/pitch target management.

It converts pilot roll/pitch input into attitude targets, including slew limits and return-to-home behavior.
Pilot pitch input keeps the driving convention separate from the body-axis sign convention: W lowers the nose by decreasing pitch target; S raises the nose by increasing pitch target.

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
heading hold
position hold
PID command output
rotor mixing
flight-mode ownership
```

---

## `rate_lock.lua`

### Does

Generic primitive for manual rate input followed by hold-target capture.

This module is used for vertical height/down hold. Heading hold is owned directly
by `control_task.lua`, because A/D is an attitude-target lookahead intent rather
than an inner-loop rate command.

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
    commandedRate = ...,
    active = ...,
    pending = ...,
    error = ...,
    state = ...,
}
```

### Used by

```text
control_task.lua
```

### Must not do

```text
decide global heading mode
decide global vertical mode
call controller
call rotor
```

---

## `lib/mathx.lua`

### Does

Provides shared math helpers, including the canonical single-axis and multi-axis projection API.

Use `mathx.component(value, axis)` for one scalar component and `mathx.project(value, axes)` for a named multi-axis projection. This keeps raw-to-body projection, heading-level horizontal projection, and coordinate component capture on one interface.

### Depends on

```text
plain tables or vector-like values with x/y/z fields
```

### Provides

```lua
mathx.component(value, axis)

mathx.project(value, {
    forward = frame.forward,
    right = frame.right,
    down = frame.down,
})
```

### Must not do

```text
read sensors
own flight modes
choose target lifetime
format telemetry snapshots
```

### Notes

`mathx.project` is only the projection primitive. The caller still owns the semantic boundary: `data_task.lua` uses it for raw-to-body sensor projection, while `control_task.lua` uses it for target capture and heading-level horizontal projection.

---

## `lib/pid.lua`

### Does

Provides a structured PID primitive with optional per-controller feedforward.

### Provides

```lua
local controller = pid.new({
    kp = ...,
    ki = ...,
    kd = ...,
    i_min = ...,
    i_max = ...,
    out_min = ...,
    out_max = ...,
    deadband = ...,
    feedforward = function(input) return 0.0 end, -- optional
})

local result = controller:update({
    target = ...,
    current = ...,
    dt = ...,
    derivative = ..., -- optional
    error = ...,      -- optional, for wrapped angles
})
```

`feedforward` is a function field on the PID object. It defaults to a zero function. PID output is clamped after `raw + ff`.

```lua
result = {
    target = ...,
    current = ...,
    error = ...,
    integral = ...,
    derivative = ...,
    output = ...,
    terms = {
        p = ...,
        i = ...,
        d = ...,
        raw = ...,    -- p + i + d
        ff = ...,     -- feedforward(input)
        output = ..., -- clamp(raw + ff)
    },
}
```

### Must not do

```text
know flight modes
read shared state
own axis semantics beyond target/current/error
```

---

## `position_hold.lua`

### Does

Primitive for horizontal position hold.

It converts navigation heading-level FRD horizontal position error and navigation heading-level FRD horizontal velocity into attitude targets or position-control terms when lateral mode is position hold.

It does not own captured raw position targets. `control_task.lua` owns target lifetime and mode transitions, then projects raw position targets and raw horizontal velocity into matching FRD vectors with `mathx.project`.

### Depends on

```text
bodyPositionError.forward
bodyPositionError.right
horizontalVelocity.forward
horizontalVelocity.right
position-hold config
dt
```

### Provides

```lua
positionHoldResult = {
    active = ...,
    position = {
        target = { right = ..., forward = ... },
        current = { right = ..., forward = ... },
        error = { right = ..., forward = ... },
    },
    velocity = {
        target = { right = ..., forward = ... },
        current = { right = ..., forward = ... },
        error = { right = ..., forward = ... },
    },
    output = {
        right = { value = ..., feedforward = ..., feedback = ... },
        forward = { value = ..., feedforward = ..., feedback = ... },
        attitude = { roll = ..., pitch = ... },
    },
}
```

Positive `output.forward.value` means a navigation-frame forward acceleration request. Since body-axis positive pitch is nose up, `output.attitude.pitch` is negative for a positive forward request.

### Must not do

```text
own global lateral mode
capture raw position targets
read state.raw.position
read state.raw.velocity
read state.body.pose.heading for horizontal projection
run final attitude PID
run rotor mixing
interpret raw sensor axes
```

### Notes

Heading-level horizontal projection belongs in `control_task.lua`, not in `data_task.lua` or `position_hold.lua`. Position error and horizontal velocity must be in the same projected frame before entering `position_hold.lua`.

---

## `controller.lua`

### Does

Converts target and body state into body commands.

This is the stabilization/control law module. It owns PID instances and control terms, not flight modes.

Roll, pitch, and yaw angle PIDs consume body-frame attitude error, not direct
Euler-angle subtraction. Positive pitch is around the body `right` axis and
means nose up. Horizontal `heading` stays outside the controller: `control_task`
uses current/target heading semantics to build `target.attitude.orientation`,
then `controller.lua` compares that orientation with the current orientation and
controls body roll, pitch, and yaw only.

Roll, pitch, and yaw use cascaded control: an angle PID produces a target body rate, then a rate PID produces the final body command. The rate PID owns the linear feedforward for commanded rate.

### Depends on

```text
target
state.frame
state.pose
state.rates
state.vertical
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
        vertical = {
            height = ...,
            speed = ...,
        },
        attitude = {
            roll = { angle = ..., rate = ... },
            pitch = { angle = ..., rate = ... },
            yaw = { angle = ..., rate = ... },
        },
    },
}
```

### Preferred API

```lua
local result = controller:update({
    target = target,
    state = {
        bodyFrame = shared.state.body.frame,
        orientation = shared.state.body.orientation,
        pose = shared.state.body.pose,
        rates = shared.state.body.rates,
        vertical = {
            height = shared.state.body.pose.height,
            speed = shared.state.raw.velocity.y,
        },
    },
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
height/heading lock logic
```

### Notes

Avoid positional command APIs. `collective, roll, yaw, pitch` ordering is too easy to misuse.
`commands.roll/pitch/yaw` are body-axis commands. If hardware mixing needs a sign adaptation, keep it in `rotor.lua`; do not invert body-axis semantics in controller code.

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
├── position_hold.lua
├── controller.lua
├── rotor.lua
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
6. control_task projects raw position targets into FRD errors before position_hold consumes them.
7. controller:update() receives { target, state, dt }.
8. rotor command API changes from positional args to setCommands(commands).
9. control_task publishes a structured telemetry snapshot directly.
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
2. Should heading-level horizontal velocity be exposed by data_task? Current answer: no.
3. Should control_task own flight mode state? Current answer: yes.
4. Should rate_lock remain? Current answer: yes, as a primitive, not as mode owner.
5. Should position_hold own lateral mode or raw target lifetime? Current answer: no; control_task owns mode/target lifetime, projects raw targets with mathx.project, and position_hold consumes FRD errors.
6. Should rotor commands use named table fields? Current answer: yes.
7. Should raw.position be visible outside data_task? Current answer: yes, but only for control_task target capture/projection and telemetry/UI, not for controller or position_hold.
```
