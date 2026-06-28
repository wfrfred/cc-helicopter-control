local allocation_control = require("control.allocation")
local attitude_control = require("control.attitude")
local frames = require("lib.frames")
local horizontal_control = require("control.horizontal")
local mathx = require("lib.mathx")
local tablex = require("lib.tablex")
local vertical_control = require("control.vertical")

--- Owns the controller target contract and maps it into child-controller inputs.
---
--- Modes choose targets by filling this contract. The composition layer
--- translates it into horizontal, vertical, attitude, and allocation controller
--- calls; child controllers stay narrow and do not know about mode ownership or
--- telemetry terms.
local controller = {}

---@class ControlController
local Controller = {}
Controller.__index = Controller

---@class ControlControllerInput
---@field state table State returned by `app.control_state.fromSensors()`.
---@field target ControlTarget
---@field dt number

---@class ControlControllerResult
---@field output { collective: number, roll: number, pitch: number, yaw: number }
---@field terms { horizontal: table, vertical: table, attitude: table, allocation: table }

-- Target contract -----------------------------------------------------------

---@class ControlTarget
---@field horizontal ControlHorizontalTarget
---@field vertical ControlVerticalTarget
---@field yaw ControlYawTarget

---@class ControlHorizontalTarget
---@field kind "position"|"attitude"
---@field position { forward: number|nil, right: number|nil }|nil Position branch only.
---@field angle { roll: number, pitch: number }|nil Attitude branch only.
---@field feedforward { position: { forward: number, right: number }|nil, velocity: { forward: number, right: number }|nil, angle: { roll: number, pitch: number }, rate: { roll: number, pitch: number } }

---@class ControlVerticalTarget
---@field position number|nil Down-axis local position. nil disables the height PID.
---@field feedforward { position: number, velocity: number }

---@class ControlYawTarget
---@field angle number|nil
---@field feedforward { angle: number, rate: number }

--- Returns an empty controller target for the selected horizontal branch.
---
--- Target contract:
---
--- - `horizontal.kind` is the only union:
---   - "position" uses the horizontal position/velocity controller to produce
---     roll/pitch attitude targets.
---   - "attitude" bypasses the horizontal position/velocity controller and uses
---     `horizontal.angle.roll/pitch` directly.
---
--- - `horizontal.position.forward/right` are heading-level local FRD positions
---   with the current aircraft position as origin. nil disables that axis'
---   position PID. In the "position" branch:
---   - `feedforward.position.forward/right` is added to the position loop output,
---     forming the velocity target.
---   - `feedforward.velocity.forward/right` is added to the velocity loop output,
---     forming the roll/pitch angle target.
---   - `feedforward.angle/rate.roll/pitch` are passed to the roll/pitch attitude
---     loops.
---
--- - `vertical.position` is a down-axis local position. nil disables the height
---   PID. `vertical.feedforward.position` is a down-axis velocity contribution;
---   `vertical.feedforward.velocity` is a collective command contribution.
---
--- - `yaw.angle` is the yaw target passed to the attitude controller. Modes must
---   set it before returning the target; using current heading is the zero-error
---   yaw target. `yaw.feedforward.angle/rate` feed the yaw attitude loops.
---@param kind "position"|"attitude"
---@return ControlTarget
function controller.target(kind)
    local target = {
        vertical = {
            position = nil,
            feedforward = {
                position = 0.0,
                velocity = 0.0,
            },
        },
        yaw = {
            angle = nil,
            feedforward = {
                angle = 0.0,
                rate = 0.0,
            },
        },
    }

    if kind == "position" then
        target.horizontal = {
            kind = kind,
            position = {
                forward = nil,
                right = nil,
            },
            feedforward = {
                position = {
                    forward = 0.0,
                    right = 0.0,
                },
                velocity = {
                    forward = 0.0,
                    right = 0.0,
                },
                angle = {
                    roll = 0.0,
                    pitch = 0.0,
                },
                rate = {
                    roll = 0.0,
                    pitch = 0.0,
                },
            },
        }

        return target
    end

    if kind == "attitude" then
        target.horizontal = {
            kind = kind,
            angle = {
                roll = 0.0,
                pitch = 0.0,
            },
            feedforward = {
                angle = {
                    roll = 0.0,
                    pitch = 0.0,
                },
                rate = {
                    roll = 0.0,
                    pitch = 0.0,
                },
            },
        }

        return target
    end

    error("target kind must be position or attitude")
end

-- Controller runtime --------------------------------------------------------

local function axisFromVector(value)
    return {
        roll = value.x,
        pitch = value.y,
        yaw = value.z,
    }
end

local function bodyAttitude(state)
    local basis = state.frames.body:basis()
    local forwardHorizontal = vector.new(basis.forward.x, 0.0, basis.forward.z)
    local horizontal = forwardHorizontal:length()

    return {
        roll = mathx.atan2(-basis.right.y, -basis.down.y),
        pitch = mathx.atan2(basis.forward.y, horizontal),
        heading = mathx.atan2(basis.forward.x, -basis.forward.z),
    }
end

---@param control table
---@return ControlController
function controller.new(control)
    return setmetatable({
        horizontal = horizontal_control.new(control),
        vertical = vertical_control.new(control),
        attitude = attitude_control.new(control),
        allocation = allocation_control.new(control),
        horizontalKind = nil,
    }, Controller)
end

function Controller:reset()
    tablex.list.each({
        self.horizontal,
        self.vertical,
        self.attitude,
        self.allocation,
    }, function(controller)
        controller:reset()
    end)
    self.horizontalKind = nil
end

local function updateHorizontal(self, state, target, dt)
    local horizontalTarget = target.horizontal

    if horizontalTarget.kind == "position" then
        if self.horizontalKind ~= "position" then
            self.horizontal:reset()
        end

        local currentVelocity = frames.frdFromVector(
            frames.level(target.yaw.angle):componentsOf(state.world.velocity)
        )
        local horizontalResult = self.horizontal:update(
            {
                velocity = {
                    forward = currentVelocity.forward,
                    right = currentVelocity.right,
                },
            },
            {
                position = horizontalTarget.position,
            },
            {
                position = horizontalTarget.feedforward.position,
                velocity = horizontalTarget.feedforward.velocity,
            },
            dt
        )

        horizontalResult.terms = tablex.record.merge({ kind = "position" }, horizontalResult.terms)
        self.horizontalKind = "position"

        return horizontalResult
    end

    if horizontalTarget.kind == "attitude" then
        self.horizontalKind = "attitude"

        return {
            output = horizontalTarget.angle,
            terms = {
                kind = "attitude",
                output = tablex.record.copy(horizontalTarget.angle),
            },
        }
    end

    error("unknown horizontal target kind: " .. tostring(horizontalTarget.kind))
end

local function updateVertical(self, state, target, dt)
    return self.vertical:update(
        {
            position = 0.0,
            velocity = state.navigation.velocity.z,
            downAxis = state.frames.body:basis().down,
        },
        {
            position = target.vertical.position,
        },
        {
            position = target.vertical.feedforward.position,
            velocity = target.vertical.feedforward.velocity,
        },
        dt
    )
end

local function updateAttitude(self, state, target, horizontalResult, dt)
    local attitudeFrame = frames.bodyFromAngles(
        horizontalResult.output.roll,
        horizontalResult.output.pitch,
        target.yaw.angle
    )

    return self.attitude:update(
        {
            orientation = state.world.orientation,
            angularVelocity = axisFromVector(state.body.angularVelocity),
        },
        {
            orientation = attitudeFrame.qWorldFromLocal,
        },
        {
            angle = {
                roll = target.horizontal.feedforward.angle.roll,
                pitch = target.horizontal.feedforward.angle.pitch,
                yaw = target.yaw.feedforward.angle,
            },
            rate = {
                roll = target.horizontal.feedforward.rate.roll,
                pitch = target.horizontal.feedforward.rate.pitch,
                yaw = target.yaw.feedforward.rate,
            },
        },
        dt
    )
end

local function updateAllocation(self, state, verticalResult, attitudeResult, dt)
    local rawCommands = {
        collective = verticalResult.output.collective,
        roll = attitudeResult.output.roll,
        pitch = attitudeResult.output.pitch,
        yaw = attitudeResult.output.yaw,
    }

    return self.allocation:update(
        {
            pose = bodyAttitude(state),
        },
        {
            commands = rawCommands,
        },
        {},
        dt
    )
end

--- Updates the controller for one control tick.
---@param input ControlControllerInput
---@return ControlControllerResult
function Controller:update(input)
    local state = input.state
    local target = input.target
    local horizontalResult = updateHorizontal(self, state, target, input.dt)
    local verticalResult = updateVertical(self, state, target, input.dt)
    local attitudeResult = updateAttitude(self, state, target, horizontalResult, input.dt)
    local allocationResult = updateAllocation(self, state, verticalResult, attitudeResult, input.dt)

    return {
        output = allocationResult.output,
        terms = {
            horizontal = horizontalResult.terms,
            vertical = verticalResult.terms,
            attitude = attitudeResult.terms,
            allocation = allocationResult.terms,
        },
    }
end

return controller
