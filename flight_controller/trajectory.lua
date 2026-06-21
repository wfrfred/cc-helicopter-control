local trajectory = {}

local Generator = {}
Generator.__index = Generator

function trajectory.new()
    return setmetatable({}, Generator)
end

local function verticalFromLock(lock)
    return {
        height = lock.target,
        speed = lock.speed,
        active = lock.active,
        pending = lock.pending,
        error = lock.error,
        source = lock.source,
    }
end

local function headingFromLock(lock)
    return {
        angle = lock.angle,
        rate = lock.rate,
        active = lock.active,
        pending = lock.pending,
        error = lock.error,
        source = lock.source,
    }
end

local function navigationVertical(navigation, currentHeight, fallback)
    if not navigation.active or navigation.target == nil or navigation.target.height == nil then
        return fallback
    end

    local targetHeight = navigation.target.height

    return {
        height = targetHeight,
        speed = 0.0,
        active = true,
        pending = false,
        error = targetHeight - currentHeight,
        source = "navigation_" .. navigation.phase,
    }
end

local function navigationHeading(navigation, currentHeading, fallback)
    if not navigation.active or navigation.target == nil or navigation.target.heading == nil then
        return fallback
    end

    local targetHeading = navigation.target.heading

    return {
        angle = targetHeading,
        rate = 0.0,
        active = true,
        pending = false,
        error = targetHeading - currentHeading,
        source = "navigation_" .. navigation.phase,
    }
end

function Generator:update(input)
    local mode = input.mode
    local state = input.state
    local vertical = verticalFromLock(input.height)
    local heading = headingFromLock(input.heading)
    local source = mode.name
    local attitude = {
        roll = nil,
        pitch = nil,
    }
    local world = {
        position = nil,
        velocity = nil,
        acceleration = nil,
    }

    if mode.name == "manual" then
        attitude.roll = mode.manualAttitude.roll
        attitude.pitch = mode.manualAttitude.pitch
    elseif mode.name == "position_hold" then
        world.position = mode.positionTarget
    elseif mode.name == "cruise" then
        world.velocity = mode.cruiseVelocity
    elseif mode.name == "navigation" then
        if mode.navigation.active and mode.navigation.target ~= nil then
            world.position = mode.navigation.target.position
        end

        vertical = navigationVertical(mode.navigation, state.body.pose.height, vertical)
        heading = navigationHeading(mode.navigation, state.navigation.heading.angle, heading)
    end

    return {
        source = source,
        attitude = attitude,
        world = world,
        vertical = vertical,
        heading = heading,
        reset = {
            horizontal = mode.reset.horizontal,
        },
        navigation = mode.navigation,
    }
end

return trajectory
