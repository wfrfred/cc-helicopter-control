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

function Cruise:update(ctx)
    return common.status(self.velocity ~= nil)
end

function Cruise:terms()
    return {
        velocity = horizontalVector(self.velocity),
        height = self.height,
        heading = self.heading,
        lock = {
            height = "cruise",
            heading = "cruise",
        },
    }
end

function Cruise:target(input)
    local feedforward = common.frdFromWorld(self.velocity, self.heading)

    return common.target({
        position = {
            down = input.state.body.pose.height - self.height,
        },
        feedforward = {
            forward = feedforward.forward,
            right = feedforward.right,
        },
        attitude = {
            yaw = self.heading,
        },
    })
end

return cruise
