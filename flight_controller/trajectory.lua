local mathx = require("lib.mathx")

local trajectory = {}

local Generator = {}
Generator.__index = Generator

function trajectory.new(heading)
    assert(type(heading) == "table", "trajectory heading config must be table")
    assert(type(heading.manual_rate) == "number", "trajectory heading.manual_rate must be number")

    return setmetatable({
        headingManualRate = heading.manual_rate,
        manualHeading = nil,
    }, Generator)
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
        error = mathx.wrapPi(targetHeading - currentHeading),
        source = "navigation_" .. navigation.phase,
    }
end

local function manualHeading(self, command, currentHeading, dt)
    local manualRateInput = command.manual.heading.rate or 0.0

    if manualRateInput == 0.0 then
        self.manualHeading = nil
        return nil
    end

    local rate = manualRateInput * self.headingManualRate

    if self.manualHeading == nil then
        self.manualHeading = currentHeading
    end

    self.manualHeading = mathx.wrapPi(self.manualHeading + rate * dt)

    return {
        angle = self.manualHeading,
        rate = rate,
        active = true,
        pending = false,
        error = mathx.wrapPi(self.manualHeading - currentHeading),
        source = "manual_trajectory",
    }
end

function Generator:update(input)
    local mode = input.mode
    local state = input.state
    local command = input.input
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

    local manual = manualHeading(self, command, state.navigation.heading.angle, input.dt)

    if manual ~= nil then
        heading = manual
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
