# Flight Controller Conventions

## Data Boundary Naming

`data_task.lua` is the boundary between raw `sublevel` data and the control
system's FRD frame. Function names in this file use these prefixes:

- `read*`: read raw data from `sublevel` only. No conversion, projection, or
  shared-state writes.
- `build*`: build structured objects from raw inputs, such as Snapshot objects,
  body frames, and body poses.
- `project*`: project a raw vector or world-space delta into another frame.
- `make*`: construct a small plain object without reading external state.
- `publish*`: write a completed Snapshot or value into `shared`.

Required frame data should not use fallback defaults. If a required field is
missing, let the code fail instead of silently substituting zero or an empty
object.

## Snapshot Shape

Use `Snapshot` for structured data captured at a boundary. A Snapshot separates
raw sensor data from body-frame control data:

```lua
local snapshot = {
    raw = {
        position = rawPosition,
        velocity = rawVelocity,
        angularVelocity = rawAngularVelocity,
    },
    body = {
        frame = bodyFrame,
        pose = bodyPose,
        velocity = bodyVelocity,
        rates = bodyRates,
    },
}
```

Control code consumes `snapshot.body.*`. Telemetry and UI may consume
`snapshot.raw.*`.
