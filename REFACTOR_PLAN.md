# Phase 2 Refactor Plan

This document tracks remaining refactor work. Completed architecture decisions live in `ARCHITECTURE.md`.

The current version is the behavioral baseline. The helicopter is flyable, and the current signs are considered correct. Refactoring must preserve existing behavior.

Principles:

- Flight/runtime code may crash on bad data or bad config instead of silently defaulting.
- `sync.lua` is the exception because it can delete local files.
- Do not split code because it might be useful later.
- Split only when a module owns an independent responsibility.
- Do not cut a cohesive pipeline into shallow wrapper files.

## Remaining Work

1. Refactor UI drawing code without changing telemetry or input behavior.

## Phase 2 Scope

Phase 2 is UI-only. It may reorganize code under `user_interface/` when the extracted module has a clear owner and reduces real duplication or file size.

Allowed work:

```text
drawing primitives
main monitor view internals
attitude display view internals
telemetry presentation mapping
```

Not allowed in Phase 2:

```text
flight-controller control-loop changes
telemetry wire-format changes
input protocol changes
rotor or actuator behavior changes
sync.lua behavior changes
```

## Target UI Boundaries

Keep task modules responsible for runtime orchestration:

```text
input_task.lua
    input loop, sequence number, rednet publication, shared.input writes

telemetry_task.lua
    rednet receive loop, shared.telemetry writes

monitor_task.lua
    main monitor allocation, draw loop, draw error handling

attitude_display.lua
    attitude monitor allocation, draw loop, draw error handling
```

Move view-specific rendering into view modules only when it makes the task modules easier to read:

```text
draw.lua
    shared monitor drawing primitives: clipping, colors, write/fill/blit helpers

monitor_view.lua
    main diagnostic display layout and widget composition

attitude_view.lua
    artificial horizon rendering

telemetry_presenter.lua
    optional display-facing telemetry mapping and formatting helpers
```

`telemetry_presenter.lua` is optional. Add it only if monitor and attitude views need shared presentation logic or if `monitor_view.lua` becomes mostly field extraction and formatting.

## Suggested Order

1. Extract low-level drawing primitives shared by `monitor_task.lua` and `attitude_display.lua`.
2. Extract the main monitor drawing body into `monitor_view.lua`; keep monitor allocation and retry/error loops in `monitor_task.lua`.
3. Extract artificial horizon drawing into `attitude_view.lua`; keep monitor allocation and retry/error loops in `attitude_display.lua`.
4. Re-evaluate telemetry presentation mapping after the view split. Extract `telemetry_presenter.lua` only if it removes meaningful duplication.

## Drawing Primitive Rules

The drawing primitive module may own:

```text
clip text to width
write text at monitor coordinates
fill spans with background color
clear monitor background
convert colors to blit characters
write blit rows safely inside monitor bounds
```

It must not own:

```text
flight telemetry semantics
layout section order
axis labels
PID display choices
attitude horizon math
monitor discovery or retry loops
```

## View Rules

`monitor_view.lua` may own:

```text
main diagnostic layout
status coloring
controller rows
PID output rows
rotor output rows
flight-state rows
footer content
```

`attitude_view.lua` may own:

```text
roll/pitch clamping for display
horizon row generation
center marker placement
attitude display colors
```

Both view modules should expose a small API:

```lua
local monitor_view = require("monitor_view")

monitor_view.draw(mon, shared)
```

```lua
local attitude_view = require("attitude_view")

attitude_view.draw(mon, shared)
```

## Acceptance Checklist

- The UI still starts from `user_interface/startup.lua` with the same four tasks.
- `monitor_task.lua` and `attitude_display.lua` still handle monitor allocation and draw-loop failures.
- Rednet protocols and telemetry table shapes are unchanged.
- Main monitor and attitude monitor output are visually equivalent before and after each extraction.
- Lua syntax checks pass for all runtime files.
