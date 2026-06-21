local env = require("tools.test_env")
env.install()

local config = require("config")
local Controller = require("control.controller")
local heading_lock = require("state.heading_lock")
local height_lock = require("state.height_lock")
local mode_state = require("state.mode_state")
local trajectory = require("trajectory")
local attitude_math = require("lib.attitude_math")
local mixer = require("hardware.mixer")

local M = {}

local function baseState()
    local frame = attitude_math.frameFromPose(0.0, 0.0, 0.0)

    return {
        raw = {},
        world = {
            position = vector.new(-213.0, 80.0, 304.0),
            velocity = vector.new(0.0, 0.0, 0.0),
        },
        body = {
            frame = frame,
            orientation = attitude_math.quaternionFromFrame(frame),
            pose = {
                roll = 0.0,
                pitch = 0.0,
                heading = 0.0,
                height = 80.0,
            },
            angular = {
                velocity = {
                    roll = 0.0,
                    pitch = 0.0,
                    yaw = 0.0,
                },
            },
        },
        navigation = {
            heading = {
                angle = 0.0,
                rate = 0.0,
            },
            velocity = {
                forward = 0.0,
                right = 0.0,
                up = 0.0,
            },
        },
        time = {
            pose = 0.0,
            velocity = 0.0,
            angularVelocity = 0.0,
        },
    }
end

local function inputCommand(roll, pitch, climb, heading, cruiseToggle)
    return {
        manual = {
            mode = "manual.attitude",
            arm = true,
            attitude = {
                roll = roll,
                pitch = pitch,
            },
            velocity = {
                forward = 0.0,
                right = 0.0,
                up = climb,
            },
            heading = {
                rate = heading,
            },
        },
        navigation = {
            action = nil,
            waypoint = nil,
        },
        event = {
            cruiseToggle = cruiseToggle == true,
            holdCapture = false,
        },
    }
end

local function makeMachines(state)
    return {
        mode = mode_state.new(state, config),
        height = height_lock.new({
            initial_target = state.body.pose.height,
            target_rate = config.control.vertical.target_rate,
            rate_deadband = config.control.vertical.lock.speed_deadband,
            relock_timeout = config.control.vertical.lock.relock_timeout,
        }),
        heading = heading_lock.new({
            initial_heading = state.navigation.heading.angle,
            lookahead_rate = config.control.heading.lookahead_rate,
            time_constant = config.control.attitude.time_constant,
            rate_deadband = config.control.heading.lock.rate_deadband,
            relock_timeout = config.control.heading.lock.relock_timeout,
        }),
        trajectory = trajectory.new(),
        controller = Controller.new(config.control),
        mixer = mixer.new(config.hardware.rotor, config.calibration.rotor),
    }
end

local function runCase(name, input, navigationCommand, mutateState)
    local state = baseState()

    if mutateState then
        mutateState(state)
    end

    local machines = makeMachines(state)
    local mode = machines.mode:update({
        input = input,
        state = state,
        navigationCommand = navigationCommand,
        dt = config.control.loop.dt,
    })
    local height = machines.height:update({
        climb = input.manual.velocity.up,
        height = state.body.pose.height,
        verticalSpeed = state.world.velocity.y,
        dt = config.control.loop.dt,
    })
    local heading = machines.heading:update({
        headingInput = input.manual.heading.rate,
        heading = state.navigation.heading.angle,
        headingRate = state.navigation.heading.rate,
        dt = config.control.loop.dt,
    })
    local target = machines.trajectory:update({
        mode = mode,
        input = input,
        state = state,
        height = height,
        heading = heading,
        dt = config.control.loop.dt,
    })
    local command = machines.controller:update({
        state = state,
        target = target,
        dt = config.control.loop.dt,
    })
    local rotor = machines.mixer:update({
        commands = command,
        phase = {
            upper = 0.0,
            lower = 0.0,
        },
    })

    return {
        name = name,
        input = input,
        expected = {
            mode = mode.name,
            height = height,
            heading = heading,
            target = target,
            command = command,
            rotor = rotor,
        },
    }
end

function M.cases()
    return {
        runCase("manual neutral", inputCommand(0.0, 0.0, 0.0, 0.0)),
        runCase("manual roll positive", inputCommand(1.0, 0.0, 0.0, 0.0)),
        runCase("manual pitch positive", inputCommand(0.0, 1.0, 0.0, 0.0)),
        runCase("manual climb positive", inputCommand(0.0, 0.0, 1.0, 0.0)),
        runCase("manual heading positive", inputCommand(0.0, 0.0, 0.0, 1.0)),
        runCase("position_hold neutral", inputCommand(0.0, 0.0, 0.0, 0.0)),
        runCase("cruise capture", inputCommand(0.0, 0.0, 0.0, 0.0, true), nil, function(state)
            state.world.velocity = vector.new(3.0, 0.0, -1.0)
        end),
        runCase(
            "navigation active target",
            inputCommand(0.0, 0.0, 0.0, 0.0),
            { action = "activate", waypoint = "home" },
            function(state)
                state.world.position = vector.new(-213.0, 90.0, 304.0)
                state.body.pose.height = 90.0
            end
        ),
        {
            name = "input stale zero input behavior",
            inputAge = config.control.input.stale_dt + 0.1,
            expected = {
                stale = true,
                manual = inputCommand(0.0, 0.0, 0.0, 0.0).manual,
            },
        },
    }
end

return M
