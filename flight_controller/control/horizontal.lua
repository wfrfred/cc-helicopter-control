local mathx = require("lib.mathx")
local pid = require("lib.pid")
local tablex = require("lib.tablex")

local horizontal = {}

---@class ControlHorizontalController
local Horizontal = {}
Horizontal.__index = Horizontal

---@param control table
---@return ControlHorizontalController
function horizontal.new(control)
    local controllers = {
        forward = {
            position = pid.new(control.pid.position.forward),
            velocity = pid.new(control.pid.velocity.forward),
        },
        right = {
            position = pid.new(control.pid.position.right),
            velocity = pid.new(control.pid.velocity.right),
        },
    }

    return setmetatable({
        control = control,
        controllers = controllers,
        velocityFeedforward = control.position_hold.velocity_feedforward,
    }, Horizontal)
end

function Horizontal:reset()
    tablex.record.each(self.controllers, function(controller)
        controller.position:reset()
        controller.velocity:reset()
    end)
end

--- Updates heading-level horizontal position/velocity control.
---@param state { velocity: { forward: number, right: number } }
---@param target { position: { forward: number|nil, right: number|nil } }
---@param feedforwardInput { position: { forward: number, right: number }, velocity: { forward: number, right: number } }
---@param dt number
---@return { output: { roll: number, pitch: number }, terms: table }
function Horizontal:update(state, target, feedforwardInput, dt)
    local currentVelocity = state.velocity
    local targetVelocity = tablex.record.copy(feedforwardInput.position)
    local positionResults = tablex.record.map(self.controllers, function(controller, axis)
        if target.position[axis] == nil then
            controller.position:reset()

            return nil
        end

        local result = controller.position:update(
            target.position[axis],
            0.0,
            dt,
            currentVelocity[axis]
        )

        targetVelocity[axis] = targetVelocity[axis] + result.output

        return result
    end)
    local velocityResults = tablex.record.map(self.controllers, function(controller, axis)
        return controller.velocity:update(targetVelocity[axis], currentVelocity[axis], dt)
    end)
    local velocityTargetFeedforward = tablex.record.map(targetVelocity, function(value, axis)
        local config = self.velocityFeedforward[axis]

        return mathx.directionalAffine(value, config.gain_neg, config.gain_pos)
    end)
    local tiltCommand = tablex.record.map(velocityResults, function(result, axis)
        return result.output + velocityTargetFeedforward[axis] + feedforwardInput.velocity[axis]
    end)
    local commands = {
        roll = mathx.clamp(
            tiltCommand.right,
            -self.control.attitude.limit.roll,
            self.control.attitude.limit.roll
        ),
        pitch = mathx.clamp(
            -tiltCommand.forward,
            -self.control.attitude.limit.pitch,
            self.control.attitude.limit.pitch
        ),
    }

    return {
        output = commands,
        terms = {
            position = tablex.record.map(self.controllers, function(_, axis)
                local result = positionResults[axis]

                return {
                    target = target.position[axis],
                    current = 0.0,
                    output = result and result.output or nil,
                    pid = result and result.terms or nil,
                }
            end),
            velocity = tablex.record.map(velocityResults, function(result, axis)
                return {
                    target = targetVelocity[axis],
                    current = currentVelocity[axis],
                    output = result.output,
                    pid = result.terms,
                }
            end),
            output = {
                roll = commands.roll,
                pitch = commands.pitch,
            },
            feedforward = {
                position = tablex.record.copy(feedforwardInput.position),
                velocity = tablex.record.copy(feedforwardInput.velocity),
                velocityTarget = tablex.record.copy(velocityTargetFeedforward),
            },
        },
    }
end

return horizontal
