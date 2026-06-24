local common = require("modes.common")
local mathx = require("lib.mathx")

local cruise = {}

local Cruise = {}
Cruise.__index = Cruise

local function horizontalVector(value)
    return vector.new(value.x, 0.0, value.z)
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

function Cruise:exit()
    self.velocity = nil
    self.height = nil
    self.heading = nil
end

function Cruise:update() end

function Cruise:terms(state)
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

function Cruise:target(ctx)
    local feedforward = common.frdFromWorld(self.velocity, self.heading)
    local target = common.target()

    target.translation.position.down = ctx.state.body.pose.height - self.height
    target.translation.feedforward.forward = feedforward.forward
    target.translation.feedforward.right = feedforward.right
    target.attitude.angle.yaw = self.heading

    return target
end

return cruise
