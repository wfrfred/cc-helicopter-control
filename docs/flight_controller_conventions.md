# Flight Controller Conventions

## Data Boundary Naming

`data_task.lua` is the boundary between raw `sublevel` data and the control
system's FRD frame. Function names in this file use these prefixes:

- `read*`: perform one `sublevel` read and return a structured Snapshot with
  `raw`, `body`, and `time` fields. No shared-state writes.
- `build*`: build derived structures from already-read raw inputs, such as body
  frames and body poses. No `sublevel` reads and no shared-state writes.
- `project*`: project a raw vector or world-space delta into another frame.
- `make*`: construct a small plain object without reading external state.

Required frame data should not use fallback defaults. If a required field is
missing, let the code fail instead of silently substituting zero or an empty
object.

## Snapshot Shape

Use `Snapshot` for structured data captured at a boundary. A Snapshot separates
raw sensor data from body-frame control data and carries the read timestamp:

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
    time = os.clock(),
}
```

Shared sensor state stores whole Snapshots, for example `shared.poseSnapshot`,
`shared.velocitySnapshot`, and `shared.ratesSnapshot`. Control code consumes
`snapshot.body.*`. Telemetry and UI may consume `snapshot.raw.*`.
