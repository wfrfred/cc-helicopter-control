local allocation_control = require("control.allocation")
local attitude_control = require("control.attitude")
local frames = require("lib.frames")
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
    local altitudePosition = nil

    if target.altitude.position ~= nil then
        altitudePosition = state.body.pose.height - target.altitude.position
    end

    return self.vertical:update(
        {
            position = state.body.pose.height,
            velocity = state.world.velocity.y,
            downAxis = state.body.frame:basis().down,
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

local function updateAttitude(self, state, target, horizontalResult, dt)
    local attitudeFrame = frames.bodyFromAngles(
        horizontalResult.output.roll,
        horizontalResult.output.pitch,
        target.yaw.angle
    )

    return self.attitude:update(
        {
            orientation = state.body.frame.qWorldFromLocal,
            angularVelocity = state.body.angular.velocity,
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
