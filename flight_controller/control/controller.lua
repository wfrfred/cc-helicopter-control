local allocation_control = require("control.allocation")
local attitude_math = require("lib.attitude_math")
local attitude_control = require("control.attitude")
local horizontal_control = require("control.horizontal")
local vertical_control = require("control.vertical")

local controller = {}

local Controller = {}
Controller.__index = Controller

function controller.new(control)
    return setmetatable({
        horizontal = horizontal_control.new(control),
        vertical = vertical_control.new(control),
        attitude = attitude_control.new(control),
        allocation = allocation_control.new(control),
        lastTerms = {},
    }, Controller)
end

local function horizontalTranslationRequested(translation)
    local position = translation.position
    local feedforward = translation.feedforward

    return position.forward ~= nil
        or position.right ~= nil
        or feedforward.forward ~= 0.0
        or feedforward.right ~= 0.0
end

local function horizontalFrame(heading)
    return {
        forward = vector.new(math.sin(heading), 0.0, -math.cos(heading)),
        right = vector.new(math.cos(heading), 0.0, math.sin(heading)),
    }
end

local function updateHorizontal(self, state, target, heading, reset, dt)
    local translation = target.translation

    if reset.horizontal then
        self.horizontal:reset()
    end

    if horizontalTranslationRequested(translation) then
        return self.horizontal:update({
            state = state,
            frame = horizontalFrame(heading),
            target = {
                position = translation.position,
            },
            feedforward = {
                velocity = {
                    forward = translation.feedforward.forward,
                    right = translation.feedforward.right,
                },
            },
            dt = dt,
        })
    end

    return self.horizontal:inactive()
end

local function desiredAttitudeAngle(target, horizontalResult, heading)
    local angle = target.attitude.angle

    return {
        roll = angle.roll or horizontalResult.output.attitude.roll,
        pitch = angle.pitch or horizontalResult.output.attitude.pitch,
        yaw = angle.yaw or heading,
    }
end

function Controller:update(input)
    local state = input.state
    local target = input.target
    local reset = input.reset or {}
    local attitudeAngle = target.attitude.angle
    local heading = attitudeAngle.yaw or state.navigation.heading.angle

    assert(
        not ((attitudeAngle.roll ~= nil or attitudeAngle.pitch ~= nil)
            and horizontalTranslationRequested(target.translation)),
        "target cannot combine horizontal translation with roll/pitch attitude angles"
    )

    local horizontal = updateHorizontal(self, state, target, heading, reset, input.dt)
    local height = nil

    if target.translation.position.down ~= nil then
        height = state.body.pose.height - target.translation.position.down
    end

    local vertical = self.vertical:update({
        state = state,
        target = {
            height = height,
        },
        feedforward = {
            velocity = -target.translation.feedforward.down,
        },
        dt = input.dt,
    })
    local attitudeAngleTarget = desiredAttitudeAngle(target, horizontal, heading)
    local attitudeFrame = attitude_math.frameFromPose(
        attitudeAngleTarget.roll,
        attitudeAngleTarget.pitch,
        attitudeAngleTarget.yaw
    )
    local attitudeCommands = self.attitude:update({
        state = state,
        target = {
            orientation = attitude_math.quaternionFromFrame(attitudeFrame):normalize(),
        },
        feedforward = target.attitude.feedforward,
        dt = input.dt,
    })
    local command = self.allocation:update({
        pose = state.body.pose,
        rawCommands = {
            collective = vertical.collective,
            roll = attitudeCommands.roll,
            pitch = attitudeCommands.pitch,
            yaw = attitudeCommands.yaw,
        },
    })

    self.lastTerms = {
        horizontal = self.horizontal:terms(),
        vertical = self.vertical:terms(),
        attitude = self.attitude:terms(),
        allocation = self.allocation:terms(),
    }

    return command
end

function Controller:terms()
    return self.lastTerms
end

return controller
