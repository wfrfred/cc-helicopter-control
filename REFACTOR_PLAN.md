# Refactor Plan

This document tracks remaining work and open design direction. Completed architecture decisions live in `ARCHITECTURE.md`.

The current version is the behavioral baseline. The helicopter is flyable, and the current signs are considered correct. Refactoring must preserve existing behavior.

Principles:

- Flight/runtime code may crash on bad data or bad config instead of silently defaulting.
- `sync.lua` is the exception because it can delete local files.
- Do not split code because it might be useful later.
- Split only when a module owns an independent responsibility.
- Do not cut a cohesive pipeline into shallow wrapper files.

## Remaining Work

1. Verify whether CC: Advanced Math can replace `quat.lua`; delete `quat.lua` only if verified.
2. Clean dead UI helpers.
3. Defer broader UI drawing split to Phase 2.

## Control Split

Completed:

```text
target_state.lua  -- height/roll/pitch target update
yaw_lock.lua      -- yaw-lock state machine
controller.lua    -- PID cascade: targets + pose -> commands + terms
```

Next:

- Verify whether CC: Advanced Math can replace `quat.lua`; delete `quat.lua` only if verified.
- Clean dead UI helpers.

### `controller.lua` Design

Why split:

`control_task.lua` currently mixes two concerns:

```text
orchestration
    timing, reading shared state, calling submodules, writing telemetry

control algorithm
    PID instances and cascade updates from target/state to control commands
```

These concerns have different cohesion. `control_task.lua` should say when each step runs. `controller.lua` should say how target and state become control commands.

Proposed API:

```lua
local controller = require("controller")

local ctrl = controller.new(config.control)

local result = ctrl:update({
    targets = targets,
    pose = pose,
    yawRate = yawRate,
    velocity = velocity,
    yaw = yawResult,
    dt = dt,
})
```

Proposed return shape:

```lua
{
    commands = {
        collective = 0.0,
        roll = 0.0,
        pitch = 0.0,
        yaw = 0.0,
    },

    terms = {
        height = {
            target = 0.0,
            current = 0.0,
            err = 0.0,
            out = 0.0,
        },

        roll = {
            target = 0.0,
            current = 0.0,
            err = 0.0,
            out = 0.0,
        },

        pitch = {
            target = 0.0,
            current = 0.0,
            err = 0.0,
            out = 0.0,
        },

        yaw = {
            target = 0.0,
            current = 0.0,
            err = 0.0,
            targetRate = 0.0,
            rate = 0.0,
            rateErr = 0.0,
            out = 0.0,
        },
    },
}
```

`control_task.lua` should remain the control-loop orchestrator. It should not disappear.

After extraction, `control_task.lua` should own:

```text
loop timing
input/state freshness checks
target_state update
yaw_lock update
controller update
rotor call
telemetry construction
```

`controller.lua` should own:

```text
PID instances
PID update order
control output limits
control terms
```

**Controller input** should receive all currently observable physical quantities, regardless of whether the current PID cascade uses them:

- `targets` — target state (height, roll, pitch)
- `pose` — current pose snapshot (pos, roll, pitch, yaw)
- `yawRate` — angular velocity around yaw axis
- `velocity` — linear velocity vector (reserved for height D term and future damping)
- `yaw` — yaw lock result (yaw_err, commanded_rate, angle_active)
- `dt` — loop delta time

The controller owns which quantities it uses. The orchestrator (`control_task`) owns reading everything from `shared` and passing it through.

**FDR coordinate system** is a hardcoded convention inside `controller.lua`. The controller computes in the flight dynamics reference frame and does not accept axis sign configuration. Axis conversions happen at I/O boundaries: `sensor_axis` in `data_task.lua`, `mixer_axis` in `rotor.lua`. The controller never sees raw sensor or hardware frames.

## Do Not Split `rotor.lua` Yet

`rotor.lua` remains one cohesive pipeline:

```text
phase read
    -> mixer signs
    -> yaw differential
    -> cyclic pitch
    -> upper/lower blade tables
    -> rednet broadcast
```

Do not extract `rotor_mixer.lua` yet.

Reason:

The mixer math and rotor transport are currently strongly connected and already isolated from PID/control state. Splitting now would add file boundaries without reducing risk.

A future `rotor_mixer.lua` is allowed only if pure mixer computation needs isolated tests or `rotor.lua` grows enough to obscure the pipeline.

## Verify CC: Advanced Math Before Removing `quat.lua`

`data_task.lua` currently uses custom quaternion helper code for orientation format conversion and vector rotation, then computes roll/pitch/yaw from rotated body axes.

CC:Sable depends on CC: Advanced Math, but it is not yet verified that `pose.orientation` is directly exposed as a native quaternion object that can rotate vectors.

In-game verification script:

```lua
local pose = sublevel.getLogicalPose()
local q = pose.orientation
local v = vector.new(0, 0, 1)

print(type(q))
print(q)

print("normalize", q.normalize)
print("mul", q.mul)
print("rotate", q.rotate)

if q.rotate then
    print("rotate:", q:rotate(v))
end

if q.normalize and q.mul then
    print("mul:", q:normalize():mul(v))
end
```

Delete `quat.lua` only if the native API can reproduce the current `quat.rotate(q, v)` behavior without changing signs.

Until then, keep `quat.lua`.

## Rotor / Actuator Boundary

Actuator controller is allowed to:

```text
receive blade output table
select configured blade index
apply local polarity
clamp or convert to PWM/redstone
display local status
```

Actuator controller is not allowed to:

```text
invert collective or yaw semantics
apply PID limits
handle rotor torque logic
make flight dynamics decisions
fix sensor-axis signs
fix mixer-axis signs
```

If negative collective reverses yaw differential authority, compensation belongs in `flight_controller/rotor.lua`, not in `actuator_controller/startup.lua`.

## UI Cleanup

UI cleanup is Phase 2.

Later possible work:

```text
drawing primitives
main monitor layout
attitude display internals
telemetry presentation mapping
```

Known cleanup candidate:

```text
user_interface/monitor_task.lua
    remove drawAxis if it has no caller
```

Do not mix broad UI cleanup with control-loop refactoring.
