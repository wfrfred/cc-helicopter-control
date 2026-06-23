local common = require("modes.common")
local mathx = require("lib.mathx")
local navigation_runtime = require("navigation")

local navigation = {}

local Navigation = {}
Navigation.__index = Navigation

local function horizontalVector(value)
    return vector.new(value.x, 0.0, value.z)
end

local function motion(state)
    return {
        worldVelocity = horizontalVector(state.world.velocity),
        verticalSpeed = state.world.velocity.y,
        headingRate = state.navigation.heading.rate,
    }
end

function navigation.new(config)
    return setmetatable({
        navigator = navigation_runtime.new(config),
        lastResult = nil,
    }, Navigation)
end

function Navigation:isActive()
    return self.navigator:isActive()
end

function Navigation:clear()
    self.lastResult = self.navigator:cancel("manual")
end

function Navigation:state()
    return self.lastResult or self.navigator:state()
end

function Navigation:update(command, state, dt)
    if command == nil or command.action == nil then
        if self.navigator:isActive() then
            self.lastResult = self.navigator:update(state, dt, motion(state))
            return self.lastResult
        end

        self.lastResult = self.navigator:state()
        return self.lastResult
    end

    local result = self.navigator:command(command, state, motion(state))

    if result.active and result.target == nil then
        result = self.navigator:update(state, dt, motion(state))
    end

    self.lastResult = result

    return result
end

function Navigation:cancelForManualInput(input)
    if not self.navigator:isActive() then
        return false
    end

    if input.manual.attitude.roll ~= 0.0
        or input.manual.attitude.pitch ~= 0.0
        or input.manual.velocity.up ~= 0.0
        or input.manual.heading.rate ~= 0.0 then
        self.lastResult = self.navigator:cancel("manual")
        return true
    end

    return false
end

local function navigationVertical(nav, currentHeight, fallback)
    if not nav.active or nav.target == nil or nav.target.height == nil then
        return fallback
    end

    local targetHeight = nav.target.height

    return {
        height = targetHeight,
        speed = 0.0,
        active = true,
        pending = false,
        error = targetHeight - currentHeight,
        source = "navigation_" .. nav.phase,
    }
end

local function navigationHeading(nav, currentHeading, fallback)
    if not nav.active or nav.target == nil or nav.target.heading == nil then
        return fallback
    end

    local targetHeading = nav.target.heading

    return {
        angle = targetHeading,
        rate = 0.0,
        active = true,
        pending = false,
        error = mathx.wrapPi(targetHeading - currentHeading),
        source = "navigation_" .. nav.phase,
    }
end

function Navigation:target(input)
    local target = common.base(input)
    local nav = input.navigation

    if nav.active and nav.target ~= nil then
        target.world.position = nav.target.position
    end

    target.vertical = navigationVertical(nav, input.state.body.pose.height, target.vertical)
    target.heading = navigationHeading(nav, input.state.navigation.heading.angle, target.heading)

    return target
end

return navigation
