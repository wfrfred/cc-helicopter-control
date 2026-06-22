local env = require("tools.test_env")
env.install()

local baseline = require("tools.fixtures.control_baseline")
local attitude_math = require("lib.attitude_math")
local config = require("config")
local Controller = require("control.controller")
local flight_state = require("state.flight_state")
local heading_lock = require("state.heading_lock")
local height_lock = require("state.height_lock")
local input_protocol = require("protocol.input")
local mode_state = require("state.mode_state")
local mixer = require("hardware.mixer")
local monitor_view = require("monitor_view")
local sensor_task = require("tasks.sensor_task")
local telemetryTerms = require("telemetry.terms")
local trajectory = require("trajectory")

local function assertNumber(path, value)
    assert(type(value) == "number", path .. " must be number")
    assert(value == value, path .. " must not be NaN")
    assert(value ~= math.huge and value ~= -math.huge, path .. " must be finite")
end

local function checkTable(path, value)
    if type(value) ~= "table" then
        return
    end

    for key, child in pairs(value) do
        local childPath = path .. "." .. tostring(key)

        if type(child) == "number" then
            assertNumber(childPath, child)
        elseif type(child) == "table" then
            checkTable(childPath, child)
        end
    end
end

local function assertEquivalent(path, expected, actual)
    if type(expected) == "number" then
        assertNumber(path, actual)
        assert(math.abs(actual - expected) <= 1.0e-6, path .. " expected " .. expected .. " got " .. actual)
        return
    end

    if type(expected) == "table" then
        assert(type(actual) == "table", path .. " must be table")

        for key, child in pairs(expected) do
            assertEquivalent(path .. "." .. tostring(key), child, actual[key])
        end

        return
    end

    assert(
        actual == expected,
        path .. " expected " .. tostring(expected) .. " got " .. tostring(actual)
    )
end

local seen = {}

for _, case in ipairs(baseline.cases()) do
    assert(type(case.name) == "string", "fixture case must have name")
    assert(seen[case.name] == nil, "duplicate fixture case: " .. case.name)
    seen[case.name] = true
    assert(type(case.expected) == "table", case.name .. " expected must be table")
    checkTable(case.name .. ".expected", case.expected)
end

local required = {
    "manual neutral",
    "manual roll positive",
    "manual pitch positive",
    "manual climb positive",
    "manual heading positive",
    "position_hold neutral",
    "cruise capture",
    "navigation active target",
    "input stale zero input behavior",
}

for _, name in ipairs(required) do
    assert(seen[name], "missing fixture: " .. name)
end

local function canonicalState()
    return {
        raw = {
            position = vector.new(-213.0, 80.0, 304.0),
            velocity = vector.new(0.0, 0.0, 0.0),
            angularVelocity = vector.new(0.0, 0.0, 0.0),
        },
        world = {
            position = vector.new(-213.0, 80.0, 304.0),
            velocity = vector.new(0.0, 0.0, 0.0),
        },
        body = {
            frame = {},
            orientation = {},
            pose = {
                height = 80.0,
                heading = 0.0,
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
            pose = 1.0,
            velocity = 1.0,
            angularVelocity = 1.0,
        },
    }
end

local function runtimeState()
    local frame = attitude_math.frameFromPose(0.0, 0.0, 0.0)

    return {
        raw = {
            position = vector.new(-213.0, 80.0, 304.0),
            velocity = vector.new(0.0, 0.0, 0.0),
            angularVelocity = vector.new(0.0, 0.0, 0.0),
        },
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
            pose = 1.0,
            velocity = 1.0,
            angularVelocity = 1.0,
        },
    }
end

local function canonicalInputFromAxes(axes, cruiseToggle)
    axes = axes or {}

    return {
        manual = {
            mode = "manual.attitude",
            arm = true,
            attitude = {
                roll = axes.roll or 0.0,
                pitch = axes.pitch or 0.0,
            },
            velocity = {
                forward = 0.0,
                right = 0.0,
                up = axes.climb or 0.0,
            },
            heading = {
                rate = axes.heading or 0.0,
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

local function axesFromInput(input)
    return {
        roll = input.manual.attitude.roll,
        pitch = input.manual.attitude.pitch,
        climb = input.manual.velocity.up,
        heading = input.manual.heading.rate,
    }
end

local function makeRuntimeMachines(state)
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
    }
end

local function runCurrentBaselineCase(case)
    if case.name == "input stale zero input behavior" then
        local input = input_protocol.defaultInput()

        return {
            stale = true,
            manualAxes = axesFromInput(input),
        }
    end

    local state = runtimeState()
    local input = canonicalInputFromAxes(case.input)
    local navigationCommand = nil
    local forceManual = case.expected.mode == "manual"

    if case.name == "cruise capture" then
        state.raw.velocity = vector.new(3.0, 0.0, -1.0)
        state.world.velocity = state.raw.velocity
        input.event.cruiseToggle = true
    elseif case.name == "navigation active target" then
        state.raw.position = vector.new(-213.0, 90.0, 304.0)
        state.world.position = state.raw.position
        state.body.pose.height = 90.0
        navigationCommand = {
            action = "activate",
            waypoint = "home",
        }
    end

    local machines = makeRuntimeMachines(state)
    local mode = nil

    if forceManual then
        machines.mode:updateManualAttitude(input, config.control.loop.dt)
        mode = {
            name = "manual",
            manualAttitude = {
                roll = machines.mode.manualRoll,
                pitch = machines.mode.manualPitch,
            },
            positionTarget = nil,
            cruiseVelocity = nil,
            navigation = {
                active = false,
                phase = "idle",
                target = nil,
            },
            reset = {
                horizontal = false,
            },
        }
    else
        mode = machines.mode:update({
            input = input,
            state = state,
            navigationCommand = navigationCommand,
            dt = config.control.loop.dt,
        })
    end
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
    local controlTerms = machines.controller:terms()

    if case.name == "position_hold neutral" then
        return {
            mode = mode.name,
            horizontalActive = controlTerms.horizontal.active,
            target = controlTerms.horizontal.output.attitude,
        }
    end

    if case.name == "cruise capture" then
        return {
            mode = mode.name,
            cruiseVelocity = mode.cruiseVelocity,
            target = controlTerms.horizontal.output.attitude,
        }
    end

    if case.name == "navigation active target" then
        return {
            mode = mode.name,
            active = mode.navigation.active,
            phase = mode.navigation.phase,
            target = {
                x = target.world.position.x,
                z = target.world.position.z,
                height = target.vertical.height,
                heading = target.heading.angle,
            },
        }
    end

    return {
        mode = mode.name,
        target = {
            roll = target.attitude.roll,
            pitch = target.attitude.pitch,
            height = target.vertical.height,
            verticalSpeed = target.vertical.speed,
            heightActive = target.vertical.active,
        },
        command = command,
    }
end

local function checkFrozenBaseline()
    for _, case in ipairs(baseline.cases()) do
        assertEquivalent(case.name .. ".expected", case.expected, runCurrentBaselineCase(case))
    end
end

local function assertOldRuntimeModuleRemoved(name)
    local ok = pcall(require, name)

    assert(not ok, "old root runtime module should not be requireable: " .. name)
end

local function checkProtocolDecode()
    local idle = input_protocol.decode(input_protocol.defaultInput())

    assert(idle.navigation.action == nil, "nil navigation action should decode as no command")

    local input = input_protocol.decode({
        manual = {
            mode = "manual.attitude",
            arm = true,
            attitude = {
                roll = 2.0,
                pitch = -2.0,
            },
            velocity = {
                forward = 0.5,
                right = -0.25,
                up = 0.25,
            },
            heading = {
                rate = -0.5,
            },
        },
        navigation = {
            action = "activate",
            waypoint = "home",
        },
        event = {
            cruiseToggle = true,
            holdCapture = true,
        },
    })

    assert(input.manual.attitude.roll == 1.0, "roll input should clamp")
    assert(input.manual.attitude.pitch == -1.0, "pitch input should clamp")
    assert(input.manual.velocity.forward == 0.5, "forward velocity should decode")
    assert(input.manual.velocity.right == -0.25, "right velocity should decode")
    assert(input.manual.velocity.up == 0.25, "up velocity should decode")
    assert(input.manual.heading.rate == -0.5, "heading input should become heading rate")
    assert(input.event.cruiseToggle == true, "cruise event should decode")
    assert(input.event.holdCapture == true, "hold capture event should decode")
    assert(input.navigation.action == "activate", "navigation action should decode")
    assert(input.navigation.waypoint == "home", "navigation waypoint should decode")
end

local function checkFlightState()
    local machine = flight_state.new()
    local waiting = machine:update({
        state = nil,
        input = input_protocol.defaultInput(),
        inputStale = false,
    })
    assert(waiting.name == "waiting_sensors", "missing sensors should wait")

    local stale = machine:update({
        state = canonicalState(),
        input = input_protocol.defaultInput(),
        inputStale = true,
    })
    assert(stale.name == "running", "ready sensors should run")
    assert(stale.reason == "input_stale_zeroed", "stale input should be reported")
end

local function checkTrajectoryNavigationOverride()
    local target = trajectory.new():update({
        mode = {
            name = "navigation",
            manualAttitude = {
                roll = 0.0,
                pitch = 0.0,
            },
            reset = {
                horizontal = false,
            },
            navigation = {
                active = true,
                phase = "climb",
                target = {
                    position = {
                        x = 10.0,
                        z = -20.0,
                    },
                    height = 120.0,
                    heading = 0.75,
                },
            },
        },
        input = input_protocol.defaultInput(),
        state = canonicalState(),
        height = {
            target = 80.0,
            speed = 0.0,
            active = true,
            pending = false,
            error = 0.0,
            source = "locked",
        },
        heading = {
            angle = 0.0,
            rate = 0.0,
            active = true,
            pending = false,
            error = 0.0,
            source = "locked",
        },
        dt = config.control.loop.dt,
    })

    assert(target.source == "navigation", "trajectory should keep navigation source")
    assert(target.world.position.x == 10.0, "navigation should set horizontal target x")
    assert(target.world.position.z == -20.0, "navigation should set horizontal target z")
    assert(target.vertical.height == 120.0, "navigation should override height")
    assert(target.vertical.source == "navigation_climb", "navigation height source should include phase")
    assert(target.heading.angle == 0.75, "navigation should override heading")
    assert(target.heading.source == "navigation_climb", "navigation heading source should include phase")
end

local function checkNavigationHeadingWrap()
    local state = canonicalState()

    state.navigation.heading.angle = 3.0

    local target = trajectory.new():update({
        mode = {
            name = "navigation",
            manualAttitude = {
                roll = 0.0,
                pitch = 0.0,
            },
            reset = {
                horizontal = false,
            },
            navigation = {
                active = true,
                phase = "turn",
                target = {
                    position = {
                        x = 0.0,
                        z = 0.0,
                    },
                    heading = -3.0,
                },
            },
        },
        input = input_protocol.defaultInput(),
        state = state,
        height = {
            target = 80.0,
            speed = 0.0,
            active = true,
            pending = false,
            error = 0.0,
            source = "locked",
        },
        heading = {
            angle = 3.0,
            rate = 0.0,
            active = true,
            pending = false,
            error = 0.0,
            source = "locked",
        },
        dt = config.control.loop.dt,
    })

    assert(math.abs(target.heading.error) < 1.0, "navigation heading error should wrap")
end

local function checkNavigationVelocityFrame()
    local velocity = sensor_task.navigationVelocity(vector.new(1.0, 2.0, 0.0), math.pi / 2)

    assert(math.abs(velocity.forward - 1.0) < 1.0e-9, "navigation forward velocity should be heading-aligned")
    assert(math.abs(velocity.right) < 1.0e-9, "navigation right velocity should be heading-aligned")
    assert(math.abs(velocity.up - 2.0) < 1.0e-9, "navigation up velocity should be world y")
end

local function checkActiveNavigationKeepsTarget()
    local state = canonicalState()
    local machine = mode_state.new(state, config)
    local input = input_protocol.defaultInput()

    state.world.position = vector.new(-213.0, 90.0, 304.0)
    state.body.pose.height = 90.0

    machine:update({
        input = input,
        state = state,
        navigationCommand = {
            action = "activate",
            waypoint = "home",
        },
        dt = config.control.loop.dt,
    })

    local nextMode = machine:update({
        input = input,
        state = state,
        navigationCommand = nil,
        dt = config.control.loop.dt,
    })

    assert(nextMode.name == "navigation", "active navigation should remain selected without a new command")
    assert(nextMode.navigation.active == true, "navigation should remain active without a new command")
    assert(type(nextMode.navigation.target) == "table", "active navigation should keep a target every tick")
    assert(type(nextMode.navigation.target.position) == "table", "active navigation target should include position")
end

local function checkActiveNavigationSelectKeepsTarget()
    local state = canonicalState()
    local machine = mode_state.new(state, config)
    local input = input_protocol.defaultInput()

    state.world.position = vector.new(-213.0, 90.0, 304.0)
    state.body.pose.height = 90.0

    machine:update({
        input = input,
        state = state,
        navigationCommand = {
            action = "activate",
            waypoint = "home",
        },
        dt = config.control.loop.dt,
    })

    local selected = machine:update({
        input = input,
        state = state,
        navigationCommand = {
            action = "select",
            waypoint = "home",
        },
        dt = config.control.loop.dt,
    })

    assert(selected.name == "navigation", "active selected navigation should remain selected")
    assert(selected.navigation.active == true, "active selected navigation should remain active")
    assert(type(selected.navigation.target) == "table", "active selected navigation should keep a target")
    assert(type(selected.navigation.target.position) == "table", "active selected navigation target should include position")
end

local function checkActiveNavigationUpdateReceivesDt()
    local originalNavigation = package.loaded["navigation"]
    local originalModeState = package.loaded["state.mode_state"]
    local observedDt = nil
    local fakeNavigator = nil

    local ok, err = pcall(function()
        package.loaded["state.mode_state"] = nil
        package.loaded["navigation"] = {
            new = function()
                fakeNavigator = {
                    isActive = function()
                        return true
                    end,
                    update = function(_, _, dt)
                        observedDt = dt

                        return {
                            active = true,
                            phase = "climb",
                            target = {
                                position = {
                                    x = 0.0,
                                    z = 0.0,
                                },
                                height = 80.0,
                                heading = 0.0,
                            },
                        }
                    end,
                    state = function()
                        return {
                            active = true,
                            phase = "climb",
                            target = nil,
                        }
                    end,
                    command = function()
                        error("unexpected navigation command")
                    end,
                }

                return fakeNavigator
            end,
        }

        local fakeModeState = require("state.mode_state")
        local machine = fakeModeState.new(runtimeState(), config)
        machine:update({
            input = input_protocol.defaultInput(),
            state = runtimeState(),
            navigationCommand = nil,
            dt = 0.123,
        })
    end)

    package.loaded["navigation"] = originalNavigation
    package.loaded["state.mode_state"] = originalModeState

    assert(ok, err)
    assert(math.abs(observedDt - 0.123) < 1.0e-9, "active navigation update should receive real dt")
end

local function checkCruiseToggleOneShot()
    local state = canonicalState()
    local machine = mode_state.new(state, config)
    local input = input_protocol.defaultInput()

    state.world.velocity = vector.new(3.0, 0.0, -1.0)
    input.event.cruiseToggle = true

    local first = machine:update({
        input = input,
        state = state,
        navigationCommand = nil,
        dt = config.control.loop.dt,
    })

    state.world.velocity = vector.new(9.0, 0.0, 9.0)

    local second = machine:update({
        input = input,
        state = state,
        navigationCommand = nil,
        dt = config.control.loop.dt,
    })

    assert(first.cruiseVelocity.x == 3.0, "first cruise toggle should capture velocity")
    assert(second.cruiseVelocity.x == 3.0, "held cruise toggle should not recapture velocity")
end

local function checkNavigationExitRelockTargets()
    local height = height_lock.new({
        initial_target = 80.0,
        target_rate = config.control.vertical.target_rate,
        rate_deadband = config.control.vertical.lock.speed_deadband,
    })
    local heading = heading_lock.new({
        initial_heading = 0.0,
        lookahead_rate = config.control.heading.lookahead_rate,
        time_constant = config.control.attitude.time_constant,
        rate_deadband = config.control.heading.lock.rate_deadband,
    })

    height:update({
        climb = 0.0,
        height = 80.0,
        verticalSpeed = 0.0,
        dt = config.control.loop.dt,
    })
    heading:update({
        headingInput = 0.0,
        heading = 0.0,
        headingRate = 0.0,
        dt = config.control.loop.dt,
    })

    local heightTarget = height:lockedTarget(91.0)
    local headingTarget = heading:lockedTarget(1.25)

    assert(heightTarget.target == 91.0, "navigation exit should recapture current height")
    assert(heightTarget.active == true, "navigation exit height target should be locked")
    assert(heightTarget.error == 0.0, "navigation exit height relock should have zero initial error")
    assert(math.abs(headingTarget.angle - 1.25) < 1.0e-9, "navigation exit should recapture heading")
    assert(headingTarget.active == true, "navigation exit heading target should be locked")
    assert(math.abs(headingTarget.error) < 1.0e-9, "navigation exit heading relock should have zero initial error")
end

local function checkNavigationExitRelockTrajectory()
    local state = canonicalState()
    local modes = mode_state.new(state, config)
    local height = height_lock.new({
        initial_target = 80.0,
        target_rate = config.control.vertical.target_rate,
        rate_deadband = config.control.vertical.lock.speed_deadband,
        relock_timeout = config.control.vertical.lock.relock_timeout,
    })
    local heading = heading_lock.new({
        initial_heading = 0.0,
        lookahead_rate = config.control.heading.lookahead_rate,
        time_constant = config.control.attitude.time_constant,
        rate_deadband = config.control.heading.lock.rate_deadband,
        relock_timeout = config.control.heading.lock.relock_timeout,
    })
    local generator = trajectory.new()
    local input = input_protocol.defaultInput()

    state.world.position = vector.new(-213.0, 90.0, 304.0)
    state.raw.position = state.world.position
    state.body.pose.height = 90.0

    modes:update({
        input = input,
        state = state,
        navigationCommand = {
            action = "activate",
            waypoint = "home",
        },
        dt = config.control.loop.dt,
    })

    state.world.position = vector.new(-213.0, 97.0, 304.0)
    state.raw.position = state.world.position
    state.body.pose.height = 97.0
    state.navigation.heading.angle = 1.25

    local mode = modes:update({
        input = input,
        state = state,
        navigationCommand = {
            action = "toggle",
            waypoint = "home",
        },
        dt = config.control.loop.dt,
    })
    local heightTarget = height:update({
        climb = input.manual.velocity.up,
        height = state.body.pose.height,
        verticalSpeed = state.world.velocity.y,
        dt = config.control.loop.dt,
    })
    local headingTarget = heading:update({
        headingInput = input.manual.heading.rate,
        heading = state.navigation.heading.angle,
        headingRate = state.navigation.heading.rate,
        dt = config.control.loop.dt,
    })

    assert(mode.transition.navigationExited == true, "mode state should report navigation exit edge")

    if mode.transition.navigationExited and input.manual.velocity.up == 0.0 then
        heightTarget = height:lockedTarget(state.body.pose.height)
    end

    if mode.transition.navigationExited and input.manual.heading.rate == 0.0 then
        headingTarget = heading:lockedTarget(state.navigation.heading.angle)
    end

    local target = generator:update({
        mode = mode,
        input = input,
        state = state,
        height = heightTarget,
        heading = headingTarget,
        dt = config.control.loop.dt,
    })

    assert(mode.name == "position_hold", "navigation toggle exit should return to position_hold")
    assert(target.vertical.height == 97.0, "navigation exit should relock target height to current height")
    assert(target.vertical.source == "locked", "navigation exit height target should use lock source")
    assert(target.vertical.error == 0.0, "navigation exit height target should start with zero error")
    assert(math.abs(target.heading.angle - 1.25) < 1.0e-9, "navigation exit should relock target heading to current heading")
    assert(target.heading.source == "locked", "navigation exit heading target should use lock source")
    assert(math.abs(target.heading.error) < 1.0e-9, "navigation exit heading target should start with zero error")
end

local function checkTelemetryPreservesConsumedCruiseEvent()
    local telemetry = telemetryTerms.running({
        now = 1.0,
        dt = config.control.loop.dt,
        input = input_protocol.defaultInput(),
        inputEvent = {
            cruiseToggle = true,
            holdCapture = false,
        },
        inputAge = 0.0,
        inputStale = false,
        inputSender = 1,
        state = canonicalState(),
        flight = {
            name = "running",
            reason = "ready",
        },
        mode = {
            name = "cruise",
        },
        height = {
            source = "locked",
        },
        heading = {
            source = "locked",
        },
        target = {
            navigation = {
                active = false,
            },
        },
        command = {
            collective = 0.0,
            roll = 0.0,
            pitch = 0.0,
            yaw = 0.0,
        },
        control = {},
        rotor = {},
    })

    assert(telemetry.input.event.cruiseToggle == true, "telemetry should preserve consumed cruise event")
    assert(type(telemetry.state.body.angular.velocity) == "table", "telemetry should expose angular velocity")
    assert(type(telemetry.control) == "table", "telemetry should expose control terms")
    assert(telemetry["out" .. "put"] == nil, "old output diagnostics should not be exposed")
    assert(telemetry["cur" .. "rent"] == nil, "old current diagnostics should not be exposed")
    assert(telemetry["er" .. "ror"] == nil, "old error diagnostics should not be exposed")
    assert(telemetry["ter" .. "ms"] == nil, "old terms diagnostics should not be exposed")
    assert(telemetry["p" .. "id"] == nil, "old pid diagnostics should not be exposed")
    assert(telemetry["tar" .. "get"] == nil, "old controller target diagnostics should not be exposed")
end

local function pidTerms()
    return {
        p = 0.0,
        i = 0.0,
        d = 0.0,
        output = 0.0,
    }
end

local function axisRate()
    return {
        rate = 0.0,
    }
end

local function positionAxis()
    return {
        forward = 0.0,
        right = 0.0,
    }
end

local function fakeMonitor(width, height)
    local monitor = {
        writes = {},
        cursor = {
            x = 1,
            y = 1,
        },
    }

    function monitor.getSize()
        return width, height
    end

    function monitor.setTextColor() end
    function monitor.setBackgroundColor() end
    function monitor.clear() end

    function monitor.setCursorPos(x, y)
        monitor.cursor.x = x
        monitor.cursor.y = y
    end

    function monitor.write(text)
        monitor.writes[#monitor.writes + 1] = {
            x = monitor.cursor.x,
            y = monitor.cursor.y,
            text = text,
        }
    end

    function monitor.blit(text)
        monitor.writes[#monitor.writes + 1] = {
            x = monitor.cursor.x,
            y = monitor.cursor.y,
            text = text,
        }
    end

    return monitor
end

local function canonicalTelemetry()
    local state = canonicalState()

    state.body.pose.roll = 0.0
    state.body.pose.pitch = 0.0
    state.body.pose.heading = 0.0
    state.body.angular.velocity = {
        roll = 0.0,
        pitch = 0.0,
        yaw = 0.0,
    }

    return {
        status = "running",
        time = 1.0,
        dt = config.control.loop.dt,
        age = {
            pose = 0.0,
            angularVelocity = 0.0,
            velocity = 0.0,
        },
        input = {
            manual = input_protocol.defaultInput().manual,
            event = {
                cruiseToggle = false,
                holdCapture = false,
            },
            age = 0.0,
            stale = false,
            sender = 1,
        },
        flight = {
            name = "running",
            reason = "ready",
        },
        mode = {
            name = "position_hold",
            navigation = {
                active = false,
            },
        },
        lock = {
            height = "locked",
            heading = "locked",
        },
        height = {
            source = "locked",
        },
        heading = {
            source = "locked",
            angle = 0.0,
            error = 0.0,
        },
        state = state,
        control = {
            vertical = {
                target = {
                    height = 80.0,
                    speed = 0.0,
                    active = true,
                    pending = false,
                },
                current = {
                    height = 80.0,
                    speed = 0.0,
                },
                error = {
                    height = 0.0,
                    speed = 0.0,
                },
                terms = {
                    height = pidTerms(),
                    speed = pidTerms(),
                    tilt = {
                        compensation = 1.0,
                        verticalFactor = 1.0,
                        uncompensated = 1.0,
                        output = 1.0,
                    },
                },
            },
            attitude = {
                commanded = {
                    roll = 0.0,
                    pitch = 0.0,
                    heading = 0.0,
                    source = "position_hold",
                },
                target = {
                    roll = axisRate(),
                    pitch = axisRate(),
                    yaw = axisRate(),
                },
                current = {
                    roll = axisRate(),
                    pitch = axisRate(),
                    yaw = axisRate(),
                    heading = {
                        angle = 0.0,
                    },
                },
                error = {
                    roll = {
                        angle = 0.0,
                        rate = 0.0,
                    },
                    pitch = {
                        angle = 0.0,
                        rate = 0.0,
                    },
                    yaw = {
                        angle = 0.0,
                        rate = 0.0,
                    },
                    heading = {
                        angle = 0.0,
                    },
                },
                terms = {
                    roll = {
                        rate = pidTerms(),
                    },
                    pitch = {
                        rate = pidTerms(),
                    },
                    yaw = {
                        rate = pidTerms(),
                    },
                },
            },
            horizontal = {
                active = true,
                worldPosition = {
                    target = {
                        x = -213.0,
                        z = 304.0,
                    },
                    current = {
                        x = -213.0,
                        z = 304.0,
                    },
                    error = {
                        x = 0.0,
                        z = 0.0,
                    },
                },
                navigationPosition = {
                    target = positionAxis(),
                    current = positionAxis(),
                    error = positionAxis(),
                },
                worldVelocity = {
                    target = {
                        x = 0.0,
                        z = 0.0,
                    },
                    current = {
                        x = 0.0,
                        z = 0.0,
                    },
                    error = {
                        x = 0.0,
                        z = 0.0,
                    },
                },
                navigationVelocity = {
                    target = positionAxis(),
                    current = positionAxis(),
                    error = positionAxis(),
                },
                output = {
                    attitude = {
                        roll = 0.0,
                        pitch = 0.0,
                    },
                },
                terms = {
                    position = {
                        forward = pidTerms(),
                        right = pidTerms(),
                    },
                    velocity = {
                        forward = pidTerms(),
                        right = pidTerms(),
                    },
                },
            },
            allocation = {
                rawCommands = {},
                allocatedCommands = {},
                debug = {},
            },
        },
        navigation = {
            active = false,
            phase = "idle",
            selected = {
                id = "home",
                name = "Home",
                position = {
                    x = -213.0,
                    y = 81.0,
                    z = 264.0,
                },
            },
            waypoint = nil,
            approach = nil,
            target = nil,
            waypoints = {
                {
                    id = "home",
                    name = "Home",
                    position = {
                        x = -213.0,
                        y = 81.0,
                        z = 264.0,
                    },
                },
            },
            reason = "selected",
        },
        command = {
            collective = 1.0,
            roll = 0.0,
            pitch = 0.0,
            yaw = 0.0,
        },
        rotor = {
            upper = {
                [1] = 1.0,
                [2] = 1.0,
                [3] = 1.0,
                [4] = 1.0,
            },
            lower = {
                [1] = 1.0,
                [2] = 1.0,
                [3] = 1.0,
                [4] = 1.0,
            },
        },
    }
end

local function checkUiTelemetryBoundary()
    local mon = fakeMonitor(80, 30)
    local shared = {
        telemetry = canonicalTelemetry(),
        telemetryTime = os.clock(),
        telemetrySender = 1,
        inputSeq = 1,
    }

    for _, page in ipairs({ "overview", "attitude", "position", "nav" }) do
        shared.monitorPage = page
        monitor_view.draw(mon, shared)
    end

    shared.telemetry.navigation = {
        active = true,
        phase = "climb",
        selected = shared.telemetry.navigation.selected,
        waypoint = shared.telemetry.navigation.selected,
        approach = nil,
        target = {
            position = {
                x = -213.0,
                y = 81.0,
                z = 264.0,
            },
            height = 97.0,
            heading = 0.0,
        },
        waypoints = shared.telemetry.navigation.waypoints,
    }
    shared.monitorPage = "nav"
    monitor_view.draw(mon, shared)

    local sawVerticalError = false

    for _, write in ipairs(mon.writes) do
        if write.text:find("%+17%.0", 1, false) ~= nil then
            sawVerticalError = true
        end
    end

    assert(sawVerticalError, "navigation page vertical error should use target height")
end

local function checkMixerFormula()
    local mix = mixer.new(config.hardware.rotor, config.calibration.rotor)
    local output = mix:update({
        commands = {
            collective = 2.0,
            roll = 3.0,
            pitch = 5.0,
            yaw = 7.0,
        },
        phase = {
            upper = 0.0,
            lower = 0.0,
        },
    })

    assertNumber("mixer upper blade 1", output.blades.upper[1])
    assertNumber("mixer lower blade 1", output.blades.lower[1])
    assert(math.abs(output.blades.upper[1] - -10.0) < 1.0e-9, "upper blade formula changed")
    assert(math.abs(output.blades.lower[1] - 14.0) < 1.0e-9, "lower blade formula changed")
end

local function checkControllerTerms()
    local state = runtimeState()
    local machines = makeRuntimeMachines(state)
    local input = canonicalInputFromAxes({
        roll = 1.0,
        pitch = 0.0,
        climb = 0.0,
        heading = 0.0,
    })
    local mode = {
        name = "manual",
        manualAttitude = {
            roll = 0.039269908169872414,
            pitch = 0.0,
        },
        navigation = {
            active = false,
            phase = "idle",
            target = nil,
        },
        reset = {
            horizontal = false,
        },
    }
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
    local command, oldDetails = machines.controller:update({
        state = state,
        target = target,
        dt = config.control.loop.dt,
    })
    local terms = machines.controller:terms()

    assert(oldDetails == nil, "controller update should return command only")
    assert(type(command.collective) == "number", "controller command should contain collective")
    assert(type(terms.horizontal) == "table", "controller terms should include horizontal")
    assert(type(terms.vertical) == "table", "controller terms should include vertical")
    assert(type(terms.attitude) == "table", "controller terms should include attitude")
    assert(type(terms.allocation) == "table", "controller terms should include allocation")
    assert(terms.output == nil, "controller terms should not duplicate final command under output")
    assert(terms.allocation["final" .. "Commands"] == nil, "controller terms should not duplicate final command")
    assert(type(terms.allocation.rawCommands) == "table", "allocation terms should include raw commands")
    assert(type(terms.allocation.allocatedCommands) == "table", "allocation terms should include allocated commands")
    assert(type(terms.allocation.debug) == "table", "allocation terms should include allocator debug")
    assert(type(terms.horizontal.terms.position.forward.output) == "number", "horizontal should own position pid terms")
    assert(type(terms.vertical.terms.height.output) == "number", "vertical should own height pid terms")
    assert(type(terms.attitude.terms.roll.rate.output) == "number", "attitude should own rate pid terms")
    assert(math.abs(terms.allocation.rawCommands.roll - terms.attitude.terms.roll.rate.output) < 1.0e-6, "rate pid output should match raw roll command")
end

checkFrozenBaseline()
checkProtocolDecode()
checkFlightState()
checkTrajectoryNavigationOverride()
checkNavigationHeadingWrap()
checkNavigationVelocityFrame()
checkActiveNavigationKeepsTarget()
checkActiveNavigationSelectKeepsTarget()
checkActiveNavigationUpdateReceivesDt()
checkCruiseToggleOneShot()
checkNavigationExitRelockTargets()
checkNavigationExitRelockTrajectory()
checkTelemetryPreservesConsumedCruiseEvent()
checkUiTelemetryBoundary()
checkMixerFormula()
checkControllerTerms()

assertOldRuntimeModuleRemoved("control_task")
assertOldRuntimeModuleRemoved("input_task")
assertOldRuntimeModuleRemoved("data_task")
assertOldRuntimeModuleRemoved("rotor")
assertOldRuntimeModuleRemoved("target_state")

print("control fixtures ok")
