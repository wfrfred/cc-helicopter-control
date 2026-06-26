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

local function updateHorizontal(self, state, target, dt)
    local horizontalTarget = target.horizontal

    if horizontalTarget.kind == "position" then
        if self.horizontalKind ~= "position" then
            self.horizontal:reset()
        end

        local currentVelocity = attitude_math.levelFrdFromWorld(state.world.velocity, target.yaw.angle)
        local horizontal = self.horizontal:update(
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

        horizontal.terms = tablex.record.merge({ kind = "position" }, horizontal.terms)
        self.horizontalKind = "position"

        return horizontal
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
    local altitudePosition = nil

    if target.altitude.position ~= nil then
        altitudePosition = state.body.pose.height - target.altitude.position
    end

    return self.vertical:update(
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
        dt
    )
end

local function updateAttitude(self, state, target, horizontal, dt)
    local attitudeFrame = attitude_math.frameFromPose(
        horizontal.output.roll,
        horizontal.output.pitch,
        target.yaw.angle
    )

    return self.attitude:update(
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
        dt
    )
end

local function updateAllocation(self, state, vertical, attitude, dt)
    local rawCommands = {
        collective = vertical.output.collective,
        roll = attitude.output.roll,
        pitch = attitude.output.pitch,
        yaw = attitude.output.yaw,
    }

    return self.allocation:update(
        {
            pose = state.body.pose,
        },
        {
            commands = rawCommands,
        },
        {},
        dt
    )
end

function Controller:update(input)
    local state = input.state
    local target = input.target
    local horizontal = updateHorizontal(self, state, target, input.dt)
    local vertical = updateVertical(self, state, target, input.dt)
    local attitude = updateAttitude(self, state, target, horizontal, input.dt)
    local allocation = updateAllocation(self, state, vertical, attitude, input.dt)

    return {
        output = allocation.output,
        terms = {
            horizontal = horizontal.terms,
            vertical = vertical.terms,
            attitude = attitude.terms,
            allocation = allocation.terms,
        },
    }
end

return controller
