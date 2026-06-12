# Helicopter Flight Control Architecture

CC:Sable Lua flight control stack, split by deployed computer:

- `user_interface/` — operator input and displays.
- `flight_controller/` — core flight controller.
- `actuator_controller/` — actuator computer for one rotor layer.
- `tools/` — development utilities, not runtime files.
- `sync.lua` — deployment helper.

Each runtime directory is the independent Lua environment for one in-game computer. Files with the same name in different runtime directories are not shared. Do not create `common/` merely because code is generally useful.

## Runtime Topology

```text
typewriter
    |
    v
user_interface/input.lua
    |
    v
user_interface/input_task.lua -- CONTROL.INPUT --> flight_controller/input_task.lua
                                                        |
                                                        v
sublevel pose/velocity   --> flight_controller/data_task.lua
                                                        |
                                                        v
                                             flight_controller/control_task.lua
                                                        |
                         +------------------------------+------------------------------+
                         |                              |                              |
                         v                              v                              v
             flight_controller/target_state.lua  flight_controller/yaw_lock.lua  flight_controller/controller.lua
                                                                                         |
                                                                                         v
                                                                               flight_controller/rotor.lua
                                                                                         |
                                                               rednet upper/lower blade tables
                                                                                         |
                                                                                         v
                                                                               actuator_controller/startup.lua
                                                                                         |
                                                                                         v
                                                                               actuator_controller/pwm.lua
                                                                                         |
                                                                                         v
                                                                                redstone outputs

flight_controller/telemetry_task.lua -- CONTROL.TELEMETRY --> user_interface/telemetry_task.lua
                                                                     |
                                                                     v
                                              user_interface/monitor_task.lua and attitude_display.lua
```

## Runtime File Tree

```text
.
├── ARCHITECTURE.md
├── REFACTOR_PLAN.md
├── sync.lua
├── user_interface/
│   ├── startup.lua
│   ├── config.lua
│   ├── input.lua
│   ├── input_task.lua
│   ├── telemetry_task.lua
│   ├── monitor_task.lua
│   ├── monitor_view.lua
│   ├── attitude_display.lua
│   ├── attitude_view.lua
│   ├── draw.lua
│   ├── display_alloc.lua
│   └── lib/
│       └── protocol.lua
├── flight_controller/
│   ├── startup.lua
│   ├── config.lua
│   ├── data_task.lua
│   ├── input_task.lua
│   ├── control_task.lua
│   ├── controller.lua
│   ├── target_state.lua
│   ├── yaw_lock.lua
│   ├── rotor.lua
│   ├── telemetry_task.lua
│   ├── telemetry_builder.lua
│   └── lib/
│       ├── mathx.lua
│       ├── pid.lua
│       └── protocol.lua
├── actuator_controller/
│   ├── startup.lua
│   ├── config.lua
│   └── pwm.lua
└── tools/
    └── scan.lua
```

## Flight Controller Modules

```text
flight_controller/
├── startup.lua              -- entry: shared table, starts parallel tasks
├── config.lua               -- executable Lua configuration
│
├── data_task.lua            -- sensor reads -> shared state
├── input_task.lua           -- rednet receive -> shared input
├── telemetry_task.lua       -- shared telemetry -> rednet broadcast
│
├── control_task.lua         -- control-loop orchestrator: timing, shared reads, submodule calls, telemetry writes
├── controller.lua           -- PID cascade: targets + state snapshot -> four-axis commands + errors
├── target_state.lua         -- height/roll/pitch target update
├── yaw_lock.lua             -- yaw-lock state machine
│
├── rotor.lua                -- rotor hardware, phase, mixer math, broadcast
├── telemetry_builder.lua    -- telemetry table construction
│
└── lib/
    ├── mathx.lua            -- math helpers: clamp, wrapPi, atan2
    ├── pid.lua              -- generic PID controller
    └── protocol.lua         -- rednet protocol name constants
```

## Module Boundaries

### Data Layer

Data-layer modules write to `shared` and do not depend on each other.

| Module | Responsibility |
| --- | --- |
| `data_task.lua` | Read CC:Sable pose, rotate body axes with the native quaternion API, read angular/linear velocity, then convert them into flight-controller-visible state. |
| `input_task.lua` | Receive rednet input, normalize it, and write `shared.input`. |
| `telemetry_task.lua` | Broadcast `shared.telemetry`. |

### Control Layer

Control-layer modules read `shared` and compute control commands.

| Module | Responsibility | Reason |
| --- | --- | --- |
| `control_task.lua` | Orchestrate loop timing, shared reads, submodule calls, rotor calls, and telemetry writes. | Separates orchestration from control algorithms. |
| `target_state.lua` | Height integration, roll/pitch slew, and roll/pitch recentering. | Target policy is independent state-transition logic. |
| `yaw_lock.lua` | Yaw-lock target, release behavior, angle wrapping, and yaw error calculation. | Yaw policy is independent from PID execution. |
| `controller.lua` | PID cascade; input targets + state snapshot, output four-axis commands and errors. | Control algorithm is independent from task orchestration. |

### Rotor / Actuator Layer

This layer converts control commands into blade outputs, then into local redstone/PWM outputs.

| Module | Responsibility | Split Decision |
| --- | --- | --- |
| `rotor.lua` | Read rotor phase, apply mixer signs, yaw differential, cyclic pitch, and broadcast upper/lower blade tables. | Keep unified; this is one cohesive pipeline. |
| `actuator_controller/startup.lua` | Receive blade table, select configured blade index, apply local polarity, and pass output to PWM. | Keep dumb. |
| `actuator_controller/pwm.lua` | Convert fractional output into redstone analog output. | Actuator-local detail. |

## Control Loop Data Flow

```text
shared.input
    -> target_state:update(...)
    -> targets { height, roll, pitch }

shared.pose
    -> yaw_lock:update(...)
    -> yawResult

targets + pose snapshot + yawResult
    -> controller:update(...)
    -> controlResult {
           commands = {
               collective,
               roll,
               pitch,
               yaw,
           },
           terms = {
               height,
               roll,
               pitch,
               yaw,
           },
       }

controlResult.commands
    -> rotor:set(...)
    -> rotor:update(...)
    -> rednet upper/lower broadcast

pose + input + controlResult.commands + controlResult.terms + rotor output
    -> telemetry_builder.running(...)
    -> shared.telemetry
```

Telemetry age fields use `poseAge` for the pose snapshot. Do not emit a `stateAge` alias.

## User Interface

`user_interface/startup.lua` starts four parallel tasks over one shared table:

- `input_task.lua`: reads typewriter input and broadcasts it.
- `telemetry_task.lua`: receives flight-controller telemetry.
- `monitor_task.lua`: main monitor allocation, draw loop, and draw error handling.
- `attitude_display.lua`: attitude monitor allocation, draw loop, and draw error handling.

UI rendering is split into view modules:

```text
draw.lua
    low-level monitor drawing primitives

monitor_view.lua
    main diagnostic monitor layout and rendering

attitude_view.lua
    artificial horizon layout and rendering
```

Input axes are discrete values in `{-1, 0, 1}`:

- `W/S` -> pitch
- `D/A` -> yaw
- `E/Q` -> roll
- `Space/Shift` -> climb

`input.lua` and `input_task.lua` stay separate:

```text
input.lua
    owns input-device interpretation

input_task.lua
    owns task loop, shared-state update, sequence number, and rednet publication
```

## Actuator Controller

`actuator_controller/startup.lua` initializes redstone sides to zero, then starts:

- receiver task: listens to the configured rotor layer.
- `pwm.run`: converts fractional output to redstone analog output.
- display task: shows local output status.

Each actuator output maps only:

```text
blade index + local polarity -> redstone side
```

Actuator controller does not own flight semantics. It does not reason about roll, pitch, yaw, collective, upper/lower torque, sensor axes, mixer axes, PID, or target state.

## Protocols

There are three rednet message families:

- `control_input`: UI -> flight controller.
- `control_telemetry`: flight controller -> UI.
- `upper` / `lower`: flight controller -> actuator controllers; messages are blade-indexed output tables.

Protocol modules contain only rednet protocol name constants. Hardware layout, blade mapping, and actuator polarity live in each node's config.

## Configuration

Each node has its own `config.lua`.

Flight-controller config groups:

```text
runtime
hardware
calibration
control
```

Actuator config describes only local output:

```text
listen layer
blade index
local polarity
redstone side
display timing
```

Negative-collective yaw differential compensation belongs in `rotor.lua`, not in the actuator controller.

## Deployment

`sync.lua` uses ordered source merging. Later sources override earlier sources.

Supported commands:

```text
sync <source> [<source> ...]
sync <source> [<source> ...] --dry-run
sync <source> [<source> ...] --config
sync <source> [<source> ...] --all
sync --update
```

Sync is recursive. Default logic sync skips `config.lua`, protects `sync.lua`, and deletes stale files that are not in the final merged file set.
