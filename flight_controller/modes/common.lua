local common = {}

--- Returns an empty controller target.
---
--- Target contract:
---
--- - A mode either controls horizontal translation or controls roll/pitch directly.
---   If `attitude.angle.roll` or `attitude.angle.pitch` is set, the mode must leave
---   `translation.position.forward/right` nil and `translation.feedforward.forward/right`
---   at 0.0. The controller asserts this contract instead of silently ignoring the
---   conflicting translation request.
---
--- - `translation.position.forward/right` are heading-level FRD position errors.
---   nil disables that position axis; `translation.feedforward.forward/right` are
---   heading-level FRD velocity feedforward.
---
--- - Vertical control is independent. `translation.position.down` is a down-axis
---   position error; nil disables height hold, while `translation.feedforward.down`
---   remains a down-axis velocity feedforward.
---
--- - Heading control is independent. `attitude.angle.yaw` explicitly controls yaw;
---   nil makes the controller use the current heading as its yaw reference.
function common.target()
    return {
        translation = {
            position = {
                forward = nil,
                right = nil,
                down = nil,
            },
            feedforward = {
                forward = 0.0,
                right = 0.0,
                down = 0.0,
            },
        },
        attitude = {
            angle = {
                roll = nil,
                pitch = nil,
                yaw = nil,
            },
            feedforward = {
                angle = {
                    roll = 0.0,
                    pitch = 0.0,
                    yaw = 0.0,
                },
                rate = {
                    roll = 0.0,
                    pitch = 0.0,
                    yaw = 0.0,
                },
            },
        },
    }
end

function common.frdFromWorld(value, heading)
    local horizontal = vector.new(value.x or 0.0, 0.0, value.z or 0.0)
    local forward = vector.new(math.sin(heading), 0.0, -math.cos(heading))
    local right = vector.new(math.cos(heading), 0.0, math.sin(heading))

    return {
        forward = horizontal:dot(forward),
        right = horizontal:dot(right),
        down = -(value.y or 0.0),
    }
end

return common
