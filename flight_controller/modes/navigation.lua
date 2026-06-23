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
    }, Navigation)
end

local function status(result)
    return {
        active = result.active,
    }
end

local function targetControl(nav, state)
    if not nav.active or nav.target == nil then
        return nil
    end

    local source = "navigation_" .. nav.phase
    local height = nav.target.height
    local heading = nav.target.heading

    return {
        height = {
            height = height,
            speed = 0.0,
            active = height ~= nil,
            pending = false,
            error = height ~= nil and height - state.body.pose.height or 0.0,
            source = source,
        },
        heading = {
            angle = heading,
            rate = 0.0,
            active = heading ~= nil,
            pending = false,
            error = heading ~= nil and mathx.wrapPi(heading - state.navigation.heading.angle) or 0.0,
            source = source,
        },
        lock = {
            height = source,
            heading = source,
        },
    }
end

local function copyTerms(value)
    local terms = {}

    for key, child in pairs(value) do
        terms[key] = child
    end

    return terms
end

function Navigation:terms(state)
    local terms = copyTerms(self.navigator:state())

    terms.waypoints = nil

    if state ~= nil then
        terms.target = self.navigator:target(state)
        terms.control = targetControl(terms, state)
    end

    return terms
end

function Navigation:enter(ctx)
    local command = ctx.command

    if command == nil or command.action == nil then
        return status(self.navigator:state())
    end

    return status(self.navigator:command(command, ctx.state, motion(ctx.state)))
end

function Navigation:update(ctx)
    return status(self.navigator:update(ctx.state, ctx.dt, motion(ctx.state)))
end

function Navigation:exit(ctx)
    if self.navigator:isActive() then
        self.navigator:cancel(ctx.reason)
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
    local nav = self.navigator:state()

    nav.target = self.navigator:target(input.state)

    if nav.active and nav.target ~= nil then
        target.world.position = nav.target.position
    end

    target.vertical = navigationVertical(nav, input.state.body.pose.height, target.vertical)
    target.heading = navigationHeading(nav, input.state.navigation.heading.angle, target.heading)

    return target
end

return navigation
