local controller = require("control.controller")
local frames = require("lib.frames")
local mathx = require("lib.mathx")

local cruise = {}

---@class CruiseMode
---@field velocity vector|nil
---@field height number|nil
---@field heading number|nil
local Cruise = {}
Cruise.__index = Cruise

local function horizontalVector(value)
    return vector.new(value.x, 0.0, value.z)
end

local function heading(state)
    local forward = state.frames.navigation:basis().forward

    return mathx.wrapPi(mathx.atan2(forward.x, -forward.z))
end

---@param self CruiseMode
---@param state ControlState
---@return table
local function buildTerms(self, state)
    local heightError = -(self.height - state.navigation.position.z)
    local headingError = mathx.wrapPi(self.heading - heading(state))

    return {
        velocity = horizontalVector(self.velocity),
        height = {
            target = -self.height,
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

---@param self CruiseMode
---@param ctx ModeContext
---@return ControlTarget
local function buildTarget(self, ctx)
    local feedforward = frames.frdFromVector(frames.level(self.heading):componentsOf(self.velocity))
    local target = controller.target("position")

    target.vertical.position = self.height - ctx.state.navigation.position.z
    target.horizontal.feedforward.position.forward = feedforward.forward
    target.horizontal.feedforward.position.right = feedforward.right
    target.yaw.angle = self.heading

    return target
end

---@return CruiseMode
function cruise.new()
    return setmetatable({
        velocity = nil,
        height = nil,
        heading = nil,
    }, Cruise)
end

---@param ctx ModeContext
function Cruise:enter(ctx)
    self.velocity = horizontalVector(ctx.state.world.velocity)
    self.height = ctx.state.navigation.position.z
    self.heading = heading(ctx.state)
end

---@param ctx ModeContext
---@return ModeResult
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
