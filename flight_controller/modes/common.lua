local common = {}

local function zeroAxes()
    return {
        roll = 0.0,
        pitch = 0.0,
        yaw = 0.0,
    }
end

function common.verticalFromLock(lock)
    return {
        height = lock.target,
        speed = lock.speed,
        active = lock.active,
        pending = lock.pending,
        error = lock.error,
        source = lock.source,
    }
end

function common.headingFromLock(lock)
    return {
        angle = lock.angle,
        rate = lock.rate,
        active = lock.active,
        pending = lock.pending,
        error = lock.error,
        source = lock.source,
    }
end

function common.base(input)
    return {
        source = input.source,
        attitude = {
            roll = nil,
            pitch = nil,
            feedforward = {
                angle = zeroAxes(),
                rate = zeroAxes(),
            },
        },
        world = {
            position = nil,
            velocity = nil,
            acceleration = nil,
        },
        vertical = input.vertical,
        heading = input.heading,
    }
end

return common
