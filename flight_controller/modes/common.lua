local common = {}

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
