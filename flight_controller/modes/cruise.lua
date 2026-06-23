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
        lastAxes = nil,
    }, Cruise)
end

local function axisTerms(self, state)
    return {
        height = {
            target = self.height,
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
    self.lastAxes = axisTerms(self, ctx.state)
end

function Cruise:exit()
    self.velocity = nil
    self.height = nil
    self.heading = nil
    self.lastAxes = nil
end

function Cruise:update(ctx)
    if ctx.current ~= "cruise" then
        return {
            active = self.velocity ~= nil,
        }
    end

    self.lastAxes = axisTerms(self, ctx.state)

    return {
        active = self.velocity ~= nil,
    }
end

function Cruise:snapshot()
    if self.velocity == nil then
        return nil
    end

    return {
        velocity = horizontalVector(self.velocity),
        height = self.height,
        heading = self.heading,
    }
end

function Cruise:terms()
    return self:snapshot()
end

function Cruise:target(input)
    local target = common.base(input)

    target.world.velocity = horizontalVector(self.velocity)
    target.vertical = {
        height = self.height,
        speed = 0.0,
        active = true,
        pending = false,
        error = self.height - input.state.body.pose.height,
        source = "cruise",
    }
    target.heading = {
        angle = self.heading,
        rate = 0.0,
        active = true,
        pending = false,
        error = mathx.wrapPi(self.heading - input.state.navigation.heading.angle),
        source = "cruise",
    }

    return target
end

function Cruise:axisTerms()
    return self.lastAxes
end

return cruise
