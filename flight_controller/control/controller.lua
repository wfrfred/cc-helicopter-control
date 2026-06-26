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

function Controller:update(input)
    local state = input.state
    local target = input.target
    local horizontalAngle = nil
    local horizontalTerms = nil

    if target.horizontal.kind == "position" and self.horizontalKind ~= "position" then
        self.horizontal:reset()
    end

    if target.horizontal.kind == "position" then
        local sinYaw = math.sin(target.yaw.angle)
        local cosYaw = math.cos(target.yaw.angle)
        local horizontal = self.horizontal:update(
            {
                velocity = {
                    forward = state.world.velocity.x * sinYaw
                        - state.world.velocity.z * cosYaw,
                    right = state.world.velocity.x * cosYaw
                        + state.world.velocity.z * sinYaw,
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
    self.horizontalKind = target.horizontal.kind

    local altitudePosition = nil
    if target.altitude.position ~= nil then
        altitudePosition = state.body.pose.height - target.altitude.position
    end

    local vertical = self.vertical:update(
        {
            position = state.body.pose.height,
            velocity = state.world.velocity.y,
            downAxis = state.body.frame.down,
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
    local attitudeFrame = attitude_math.frameFromPose(
        horizontalAngle.roll,
        horizontalAngle.pitch,
        target.yaw.angle
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
