local env = require("tools.test_env")
env.install()

local config = require("config")
local Controller = require("controller")
local target_state = require("target_state")
local position_hold = require("position_hold")
local rate_lock = require("rate_lock")
local navigation = require("navigation")
local attitude_math = require("lib.attitude_math")
local mathx = require("lib.mathx")

local M = {}

local function frame(roll, pitch, heading)
    return attitude_math.frameFromPose(roll, pitch, heading)
end

local function targetOrientation(roll, pitch, heading)
    return attitude_math.quaternionFromFrame(frame(roll, pitch, heading)):normalize()
end

local function controllerResult(attitude, vertical, state)
    local orientation = targetOrientation(state.pose.roll, state.pose.pitch, state.pose.heading)
    local target = targetOrientation(attitude.roll, attitude.pitch, vertical.heading or state.pose.heading)
    local controller = Controller.new(config.control)

    return controller:update({
        state = {
            bodyFrame = frame(state.pose.roll, state.pose.pitch, state.pose.heading),
            orientation = orientation,
            pose = state.pose,
            rates = state.rates,
            vertical = {
                height = state.pose.height,
                speed = state.verticalSpeed,
            },
        },
        target = {
            attitude = {
                roll = attitude.roll,
                pitch = attitude.pitch,
                source = attitude.source,
                orientation = target,
                fullOrientation = target,
                reducedOrientation = target,
                yawPriority = 1.0,
            },
            vertical = {
                height = vertical.height,
                speed = vertical.speed,
                active = vertical.active,
                pending = false,
                error = vertical.height - state.pose.height,
                source = vertical.source,
            },
        },
        dt = config.control.loop.dt,
    })
end

local baseState = {
    pose = {
        roll = 0.0,
        pitch = 0.0,
        heading = 0.0,
        height = 80.0,
    },
    rates = {
        roll = 0.0,
        pitch = 0.0,
        yaw = 0.0,
    },
    verticalSpeed = 0.0,
}

local function manualCase(name, controls)
    local manual = target_state.new(baseState.pose, config.control)
    manual:update(controls, config.control.loop.dt)

    local heightLock = rate_lock.new({
        initial_target = baseState.pose.height,
        target_rate = config.control.vertical.target_rate,
        rate_deadband = config.control.vertical.lock.speed_deadband,
        relock_timeout = config.control.vertical.lock.relock_timeout,
    })
    local vertical = heightLock:update(controls.climb, baseState.pose.height, 0.0, config.control.loop.dt)
    local attitude = manual:target("manual")
    local result = controllerResult(attitude, {
        height = vertical.target,
        speed = vertical.commandedRate,
        active = vertical.active,
        source = vertical.state,
    }, baseState)

    return {
        name = name,
        input = controls,
        expected = {
            mode = "manual",
            target = {
                roll = attitude.roll,
                pitch = attitude.pitch,
                height = vertical.target,
                verticalSpeed = vertical.commandedRate,
                heightActive = vertical.active,
            },
            command = result.commands,
        },
    }
end

local function positionHoldCase()
    local hold = position_hold.new(config.control)
    local result = hold:update(
        { x = 0.0, z = 0.0 },
        { x = 0.0, z = 0.0 },
        baseState.pose.heading,
        config.control.loop.dt
    )
    local command = controllerResult({
        roll = result.output.attitude.roll,
        pitch = result.output.attitude.pitch,
        source = "position_hold",
    }, {
        height = baseState.pose.height,
        speed = 0.0,
        active = true,
        source = "locked",
    }, baseState)

    return {
        name = "position_hold neutral",
        expected = {
            mode = "position_hold",
            positionHoldActive = result.active,
            target = {
                roll = result.output.attitude.roll,
                pitch = result.output.attitude.pitch,
            },
            command = command.commands,
        },
    }
end

local function cruiseCase()
    local hold = position_hold.new(config.control)
    local velocity = { x = 3.0, z = -1.0 }
    local result = hold:updateVelocity(velocity, velocity, baseState.pose.heading, config.control.loop.dt)

    return {
        name = "cruise capture",
        expected = {
            mode = "cruise",
            cruiseVelocity = velocity,
            target = {
                roll = result.output.attitude.roll,
                pitch = result.output.attitude.pitch,
            },
        },
    }
end

local function navigationCase()
    local navigator = navigation.new(config.navigation)
    local result = navigator:command(
        { action = "activate", waypoint = "home" },
        {
            raw = {
                position = vector.new(-213, 90, 304),
            },
            body = {
                pose = {
                    heading = 0.0,
                },
            },
        },
        {
            worldVelocity = { x = 0.0, z = 0.0 },
            verticalSpeed = 0.0,
            headingRate = 0.0,
        }
    )

    return {
        name = "navigation active target",
        expected = {
            mode = "navigation",
            active = result.active,
            phase = result.phase,
            target = {
                x = result.target.position.x,
                z = result.target.position.z,
                height = result.target.height,
                heading = mathx.wrapPi(result.target.heading),
            },
        },
    }
end

function M.cases()
    return {
        manualCase("manual neutral", { roll = 0.0, pitch = 0.0, heading = 0.0, climb = 0.0 }),
        manualCase("manual roll positive", { roll = 1.0, pitch = 0.0, heading = 0.0, climb = 0.0 }),
        manualCase("manual pitch positive", { roll = 0.0, pitch = 1.0, heading = 0.0, climb = 0.0 }),
        manualCase("manual climb positive", { roll = 0.0, pitch = 0.0, heading = 0.0, climb = 1.0 }),
        manualCase("manual heading positive", { roll = 0.0, pitch = 0.0, heading = 1.0, climb = 0.0 }),
        positionHoldCase(),
        cruiseCase(),
        navigationCase(),
        {
            name = "input stale zero input behavior",
            inputAge = config.control.input.stale_dt + 0.1,
            expected = {
                stale = true,
                controls = { roll = 0.0, pitch = 0.0, heading = 0.0, climb = 0.0 },
            },
        },
    }
end

return M
