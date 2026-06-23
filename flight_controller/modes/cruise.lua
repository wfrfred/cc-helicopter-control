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

local function targetControl(self, state)
    return {
        height = {
            height = self.height,
            speed = 0.0,
            active = true,
            pending = false,
            error = self.height - state.body.pose.height,
            source = "cruise",
        },
        heading = {
            angle = self.heading,
            rate = 0.0,
            active = true,
            pending = false,
            error = mathx.wrapPi(self.heading - state.navigation.heading.angle),
            source = "cruise",
        },
        lock = {
            height = "cruise",
            heading = "cruise",
        },
    }
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
    return {
        active = self.velocity ~= nil,
    }
end

function Cruise:terms(state)
    local terms = {
        velocity = horizontalVector(self.velocity),
        height = self.height,
        heading = self.heading,
    }

    if state ~= nil then
        terms.control = targetControl(self, state)
    end

    return terms
end

function Cruise:target(input)
    local target = common.base(input)
    local terms = targetControl(self, input.state)

    target.world.velocity = horizontalVector(self.velocity)
    target.vertical = terms.height
    target.heading = terms.heading

    return target
end

return cruise
