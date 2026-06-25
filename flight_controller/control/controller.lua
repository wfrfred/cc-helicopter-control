local allocation_control = require("control.allocation")
local attitude_math = require("lib.attitude_math")
local attitude_control = require("control.attitude")
local horizontal_control = require("control.horizontal")
local tablex = require("lib.tablex")
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
end

function Controller:update(input)
    local state = input.state
    local target = input.target
    local reset = input.reset or {}
    local heading = target.yaw.angle or state.navigation.heading.angle
    local horizontalAngle = nil
    local horizontalTerms = nil

    if reset.horizontal then
        self.horizontal:reset()
    end

    if target.horizontal.kind == "position" then
        local sinHeading = math.sin(heading)
        local cosHeading = math.cos(heading)
        local horizontal = self.horizontal:update(
            {
                position = {
                    forward = 0.0,
                    right = 0.0,
                },
                velocity = {
                    forward = state.world.velocity.x * sinHeading
                        - state.world.velocity.z * cosHeading,
                    right = state.world.velocity.x * cosHeading
                        + state.world.velocity.z * sinHeading,
                },
            },
            {
                position = target.horizontal.position,
            },
            {
                position = target.horizontal.feedforward.position,
                velocity = target.horizontal.feedforward.velocity,
            },
            input.dt
        )

        horizontalAngle = horizontal.output.angle
        horizontalTerms = tablex.record.merge({ kind = "position" }, horizontal.terms)
    elseif target.horizontal.kind == "attitude" then
        horizontalAngle = target.horizontal.angle
        horizontalTerms = {
            kind = "attitude",
            output = {
                angle = tablex.record.copy(horizontalAngle),
            },
        }
    else
        error("unknown horizontal target kind: " .. tostring(target.horizontal.kind))
    end

    local altitudePosition = nil
    if target.altitude.position ~= nil then
        altitudePosition = state.body.pose.height - target.altitude.position
    end

    local vertical = self.vertical:update(
        {
            position = state.body.pose.height,
            velocity = state.world.velocity.y,
            attitude = {
                roll = state.body.pose.roll,
                pitch = state.body.pose.pitch,
            },
        },
        {
            position = altitudePosition,
        },
        {
            position = -target.altitude.feedforward.position,
            velocity = target.altitude.feedforward.velocity,
        },
        input.dt
    )
    local attitudeAngleTarget = {
        roll = horizontalAngle.roll,
        pitch = horizontalAngle.pitch,
        yaw = heading,
    }
    local attitudeFrame = attitude_math.frameFromPose(
        attitudeAngleTarget.roll,
        attitudeAngleTarget.pitch,
        attitudeAngleTarget.yaw
    )
    local attitudeCommands = self.attitude:update(
        {
            orientation = state.body.orientation,
            angularVelocity = state.body.angular.velocity,
        },
        {
            orientation = attitude_math.quaternionFromFrame(attitudeFrame):normalize(),
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
        input.dt
    )
    local rawCommands = {
        collective = vertical.output.collective,
        roll = attitudeCommands.output.roll,
        pitch = attitudeCommands.output.pitch,
        yaw = attitudeCommands.output.yaw,
    }
    local allocation = self.allocation:update(
        {
            pose = state.body.pose,
        },
        {
            commands = rawCommands,
        },
        {},
        input.dt
    )

    return {
        output = allocation.output,
        terms = {
            horizontal = horizontalTerms,
            vertical = vertical.terms,
            attitude = attitudeCommands.terms,
            allocation = allocation.terms,
        },
    }
end

return controller
