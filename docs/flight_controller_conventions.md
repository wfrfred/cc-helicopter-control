# Flight Controller Conventions

## Data Boundary Naming

`data_task.lua` is the boundary between raw `sublevel` data and the control
system's FRD frame. Function names in this file use these prefixes:

- `read*`: perform one `sublevel` read and return the raw data plus derived
  body-frame data for that read. No shared-state writes.
- `build*`: build derived structures from already-read raw inputs, such as body
  frames and body poses. No `sublevel` reads and no shared-state writes.
- `project*`: project a raw vector or world-space delta into another frame.
- `make*`: construct a small plain object without reading external state.

Required frame data should not use fallback defaults. If a required field is
missing, let the code fail instead of silently substituting zero or an empty
object.

## Lua Naming

Avoid local aliases for short config paths. Prefer reading `config.control`
directly over introducing aliases such as:

```lua
local CONTROL = config.control
```

If a repeated expression is genuinely long enough to deserve a local name, use a
normal runtime-value name such as `controlConfig`, not an all-caps name. All-caps
names imply constants, and config tables or runtime references are not constants.

Prefer structured objects over repeated name prefixes. If a value is already a
coherent table such as `commands`, keep and pass that table instead of splitting
it into fields such as `collective_cmd`, `roll_cmd`, `pitch_cmd`, and `yaw_cmd`.

## State Shape

`data_task.lua` publishes one shared sensor state. The state separates raw
`sublevel` data from body-frame control data and tracks each read timestamp:

```lua
shared.state = {
    raw = {
        position = ...,
        orientation = ...,
        velocity = ...,
        angularVelocity = ...,
    },

    body = {
        pose = {
            down = ...,
            roll = ...,
            pitch = ...,
            yaw = ...,
        },

        velocity = {
            forward = ...,
            right = ...,
            down = ...,
        },

        rates = {
            roll = ...,
            pitch = ...,
            yaw = ...,
        },
    },

    time = {
        pose = ...,
        velocity = ...,
        rates = ...,
    },
}
```

Control code consumes `shared.state.body.*`. Raw xyz data stays inside
`data_task.lua` unless a boundary layer such as telemetry, UI, or navigation
needs it. Navigation projects raw position targets into FRD error vectors before
position hold consumes them.

Keep `body.velocity` limited to FRD components. Derived display fields such as
total speed, horizontal speed, and vertical speed are computed where they are
used.
