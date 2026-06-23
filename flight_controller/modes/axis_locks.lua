local heading_lock = require("state.heading_lock")
local height_lock = require("state.height_lock")

local axis_locks = {}

local Locks = {}
Locks.__index = Locks

function axis_locks.new(initialState, control)
    local self = setmetatable({
        height = height_lock.new({
            initial_target = initialState.body.pose.height,
            target_rate = control.vertical.target_rate,
            rate_deadband = control.vertical.lock.speed_deadband,
            relock_timeout = control.vertical.lock.relock_timeout,
        }),
        heading = heading_lock.new({
            initial_heading = initialState.navigation.heading.angle,
            target_rate = control.heading.target_rate,
            rate_deadband = control.heading.lock.rate_deadband,
            relock_timeout = control.heading.lock.relock_timeout,
        }),
        lastHeight = nil,
        lastHeading = nil,
    }, Locks)

    self.lastHeight = self.height:lockedTarget(initialState.body.pose.height)
    self.lastHeading = self.heading:lockedTarget(initialState.navigation.heading.angle)

    return self
end

function Locks:enter(ctx)
    local manual = ctx.input.manual

    if manual.velocity.up == 0.0 then
        self.lastHeight = self.height:lockedTarget(ctx.state.body.pose.height)
    end

    if manual.heading.rate == 0.0 then
        self.lastHeading = self.heading:lockedTarget(ctx.state.navigation.heading.angle)
    end
end

function Locks:update(ctx)
    local manual = ctx.input.manual
    local state = ctx.state

    self.lastHeight = self.height:update({
        climb = manual.velocity.up,
        height = state.body.pose.height,
        verticalSpeed = state.world.velocity.y,
        dt = ctx.dt,
    })
    self.lastHeading = self.heading:update({
        headingRateInput = manual.heading.rate,
        heading = state.navigation.heading.angle,
        headingRate = state.navigation.heading.rate,
        dt = ctx.dt,
    })
end

function Locks:verticalTarget()
    local height = self.lastHeight

    return {
        height = height.target,
        speed = height.speed,
        active = height.active,
        pending = height.pending,
        error = height.error,
        source = height.source,
    }
end

function Locks:headingTarget()
    local heading = self.lastHeading

    return {
        angle = heading.angle,
        rate = heading.rate,
        active = heading.active,
        pending = heading.pending,
        error = heading.error,
        source = heading.source,
    }
end

function Locks:terms()
    return {
        height = self.lastHeight,
        heading = self.lastHeading,
        lock = {
            height = self.lastHeight.source,
            heading = self.lastHeading.source,
        },
    }
end

return axis_locks
