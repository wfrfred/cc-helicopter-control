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

function Navigation:terms()
    return self.lastResult or self.navigator:state()
end

function Navigation:enter(ctx)
    local command = ctx.command

    if command == nil or command.action == nil then
        self.lastResult = self.navigator:state()
        return {
            active = self.lastResult.active,
        }
    end

    self.lastResult = self.navigator:command(command, ctx.state, motion(ctx.state))

    return {
        active = self.lastResult.active,
    }
end

function Navigation:update(ctx)
    if ctx.current ~= "navigation" then
        return {
            active = self:terms().active,
        }
    end

    self.lastResult = self.navigator:update(ctx.state, ctx.dt, motion(ctx.state))

    return {
        active = self.lastResult.active,
    }
end

function Navigation:exit(ctx)
    if self:terms().active then
        self.lastResult = self.navigator:cancel(ctx.reason)
    end
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
