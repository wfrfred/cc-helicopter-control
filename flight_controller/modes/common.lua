local common = {}

--- Returns an empty controller target for the selected horizontal branch.
---
--- Target contract:
---
--- - `horizontal.kind` is the only union:
---   - "position" uses the horizontal position/velocity controller to produce
---     roll/pitch attitude targets.
---   - "attitude" bypasses the horizontal position/velocity controller and uses
---     `horizontal.angle.roll/pitch` directly.
---
--- - `horizontal.position.forward/right` are heading-level local FRD positions
---   with the current aircraft position as origin. nil disables that axis'
---   position PID. In the "position" branch:
---   - `feedforward.position.forward/right` is added to the position loop output,
---     forming the velocity target.
---   - `feedforward.velocity.forward/right` is added to the velocity loop output,
---     forming the roll/pitch angle target.
---   - `feedforward.angle/rate.roll/pitch` are passed to the roll/pitch attitude
---     loops.
---
--- - `altitude.position` is a down-axis local position. nil disables the height
---   PID. `altitude.feedforward.position` is a down-axis velocity contribution;
---   `altitude.feedforward.velocity` is a collective command contribution.
---
--- - `yaw.angle` is the yaw target passed to the attitude controller. Modes must
---   set it before returning the target; using current heading is the zero-error
---   yaw target. `yaw.feedforward.angle/rate` feed the yaw attitude loops.
function common.target(kind)
    local target = {
        altitude = {
            position = nil,
            feedforward = {
                position = 0.0,
                velocity = 0.0,
            },
        },
        yaw = {
            angle = nil,
            feedforward = {
                angle = 0.0,
                rate = 0.0,
            },
        },
    }

    if kind == "position" then
        target.horizontal = {
            kind = kind,
            position = {
                forward = nil,
                right = nil,
            },
            feedforward = {
                position = {
                    forward = 0.0,
                    right = 0.0,
                },
                velocity = {
                    forward = 0.0,
                    right = 0.0,
                },
                angle = {
                    roll = 0.0,
                    pitch = 0.0,
                },
                rate = {
                    roll = 0.0,
                    pitch = 0.0,
                },
            },
        }

        return target
    end

    if kind == "attitude" then
        target.horizontal = {
            kind = kind,
            angle = {
                roll = 0.0,
                pitch = 0.0,
            },
            feedforward = {
                angle = {
                    roll = 0.0,
                    pitch = 0.0,
                },
                rate = {
                    roll = 0.0,
                    pitch = 0.0,
                },
            },
        }

        return target
    end

    error("target kind must be position or attitude")
end

return common
