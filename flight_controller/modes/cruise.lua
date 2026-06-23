local common = require("modes.common")

local cruise = {}

local Cruise = {}
Cruise.__index = Cruise

local function horizontalVector(value)
    return vector.new(value.x, 0.0, value.z)
end

function cruise.new()
    return setmetatable({
        velocity = nil,
    }, Cruise)
end

function Cruise:enter(ctx)
    self.velocity = horizontalVector(ctx.state.world.velocity)
end

function Cruise:exit()
    self.velocity = nil
end

function Cruise:update(ctx)
    if ctx.current ~= "cruise" then
        return {
            active = self.velocity ~= nil,
        }
    end

    return {
        active = self.velocity ~= nil,
    }
end

function Cruise:snapshot()
    if self.velocity == nil then
        return nil
    end

    return horizontalVector(self.velocity)
end

function Cruise:terms()
    return self:snapshot()
end

function Cruise:target(input)
    local target = common.base(input)

    target.world.velocity = self:snapshot()

    return target
end

return cruise
