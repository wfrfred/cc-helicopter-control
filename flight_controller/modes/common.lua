local common = {}

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
        source = input.mode.name,
        attitude = {
            roll = nil,
            pitch = nil,
        },
        world = {
            position = nil,
            velocity = nil,
            acceleration = nil,
        },
        vertical = input.vertical,
        heading = input.heading,
        reset = {
            horizontal = input.mode.reset.horizontal,
        },
        navigation = input.mode.navigation,
    }
end

return common
