local common = require("modes.common")
local mathx = require("lib.mathx")

local cruise = {}

local Cruise = {}
Cruise.__index = Cruise

local function horizontalVector(value)
    return vector.new(value.x, 0.0, value.z)
end

local function buildTerms(self, state)
    local heightError = state == nil and 0.0 or self.height - state.body.pose.height
    local headingError = state == nil and 0.0
        or mathx.wrapPi(self.heading - state.navigation.heading.angle)

    return {
        velocity = horizontalVector(self.velocity),
        height = {
            target = self.height,
            rate = 0.0,
            error = heightError,
        },
        heading = {
            target = self.heading,
            rate = 0.0,
            error = headingError,
        },
    }
end

local function buildTarget(self, ctx)
    local feedforward = common.frdFromWorld(self.velocity, self.heading)
    local target = common.target("position")

    target.altitude.position = ctx.state.body.pose.height - self.height
    target.horizontal.feedforward.position.forward = feedforward.forward
    target.horizontal.feedforward.position.right = feedforward.right
    target.yaw.angle = self.heading

    return target
end

function cruise.new()
    return setmetatable({
        velocity = nil,
        height = nil,
        heading = nil,
    }, Cruise)
end

function Cruise:enter(ctx)
    self.velocity = horizontalVector(ctx.state.world.velocity)
    self.height = ctx.state.body.pose.height
    self.heading = mathx.wrapPi(ctx.state.navigation.heading.angle)
end

function Cruise:update(ctx)
    return {
        target = buildTarget(self, ctx),
        terms = buildTerms(self, ctx.state),
    }
end

function Cruise:exit()
    self.velocity = nil
    self.height = nil
    self.heading = nil
end

return cruise
