local env = require("tools.test_env")
env.install()

local baseline = require("tools.fixtures.control_baseline")
local attitude_math = require("lib.attitude_math")
local config = require("config")
local Controller = require("control.controller")
local flight_state = require("state.flight_state")
local input_protocol = require("protocol.input")
local mode_state = require("state.mode_state")
local mixer = require("hardware.mixer")
local monitor_view = require("monitor_view")
local sensor_task = require("tasks.sensor_task")
local telemetryTerms = require("telemetry.terms")

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

local function assertClose(path, actual, expected)
    assertNumber(path, actual)
    assert(math.abs(actual - expected) <= 1.0e-9, path .. " expected " .. expected .. " got " .. actual)
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
        controller = Controller.new(config.control),
    }
end

local function modeTarget(machine, request)
    return machine:target({
        input = request.input,
        state = request.state,
        dt = request.dt,
    })
end

local function runModeUpdate(machine, request)
    return machine:update({
        input = request.input,
        state = request.state,
        navigationCommand = request.navigationCommand,
        dt = request.dt,
    })
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
        machines.mode.name = "manual"
        machines.mode.modes.manual:update({
            input = input,
            state = state,
            dt = config.control.loop.dt,
            current = "manual",
        })
        machines.mode.name = "manual"
        mode = {
            name = "manual",
            transition = {
                navigationExited = false,
            },
        }
    else
        if case.name == "cruise capture" then
            machines.mode.name = "manual"
        end

        mode = runModeUpdate(machines.mode, {
            input = input,
            state = state,
            navigationCommand = navigationCommand,
            dt = config.control.loop.dt,
        })
    end

    local target = modeTarget(machines.mode, {
        input = input,
        state = state,
        dt = config.control.loop.dt,
    })
    local modeTerms = machines.mode:terms()
    local command = machines.controller:update({
        state = state,
        target = target,
        reset = modeTerms.reset,
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
            cruise = {
                x = modeTerms.cruise.velocity.x,
                z = modeTerms.cruise.velocity.z,
                height = modeTerms.cruise.height,
                heading = modeTerms.cruise.heading,
            },
            target = controlTerms.horizontal.output.attitude,
        }
    end

    if case.name == "navigation active target" then
        return {
            mode = mode.name,
            active = modeTerms.navigation.active,
            phase = modeTerms.navigation.phase,
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

    local ok = pcall(function()
        input_protocol.decode({
            navigation = {
                action = "sel" .. "ect",
                waypoint = "home",
            },
        })
    end)

    assert(not ok, "runtime input should reject UI-only navigation select")

    ok = pcall(function()
        input_protocol.decode({
            navigation = {
                action = "tog" .. "gle",
                waypoint = "home",
            },
        })
    end)

    assert(not ok, "runtime input should reject UI-only navigation toggle")
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

local function checkModeUpdateExposesStatusOnly()
    local machine = mode_state.new(canonicalState(), config)
    local result = runModeUpdate(machine, {
        input = input_protocol.defaultInput(),
        state = canonicalState(),
        navigationCommand = nil,
        dt = config.control.loop.dt,
    })
    local oldManual = "manual" .. "Attitude"
    local oldPosition = "position" .. "Target"
    local oldCruise = "cruise" .. "Velocity"

    assert(result.name == "position_hold", "mode update should expose selected mode")
    assert(type(result.transition) == "table", "mode update should expose transition status")
    assert(result[oldManual] == nil, "mode update should not expose manual target internals")
    assert(result[oldPosition] == nil, "mode update should not expose hold target internals")
    assert(result[oldCruise] == nil, "mode update should not expose cruise target internals")
    assert(result.navigation == nil, "mode update should not expose navigation telemetry")
    assert(result.reset == nil, "mode update should not expose controller reset")
end

local function checkModeTermsSnapshotsAreCopied()
    local state = canonicalState()
    local machine = mode_state.new(state, config)
    local input = canonicalInputFromAxes(nil, true)

    state.world.velocity = vector.new(3.0, 0.0, -1.0)
    machine.name = "manual"
    runModeUpdate(machine, {
        input = input,
        state = state,
        navigationCommand = nil,
        dt = config.control.loop.dt,
    })

    local first = machine:terms()

    first.manual.roll = 99.0
    first.position_hold.x = 99.0
    first.cruise.velocity.x = 99.0

    local second = machine:terms()

    assert(second.manual.roll ~= 99.0, "manual terms should be copied")
    assert(second.position_hold.x ~= 99.0, "position_hold terms should be copied")
    assert(second.cruise.velocity.x ~= 99.0, "cruise terms should be copied")
end

local function checkModeTargetNavigationOverride()
    local machine = mode_state.new(canonicalState(), config)
    machine.name = "navigation"
    machine.lastNavigation = {
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
    }

    local target = modeTarget(machine, {
        input = input_protocol.defaultInput(),
        state = canonicalState(),
        dt = config.control.loop.dt,
    })

    assert(target.source == "navigation", "controller target should keep navigation source")
    assert(target.world.position.x == 10.0, "navigation should set horizontal target x")
    assert(target.world.position.z == -20.0, "navigation should set horizontal target z")
    assert(target.attitude.feedforward.angle.roll == 0.0, "controller target should default roll angle feedforward")
    assert(target.attitude.feedforward.rate.yaw == 0.0, "controller target should default yaw rate feedforward")
    assert(target.vertical.height == 120.0, "navigation should override height")
    assert(target.vertical.source == "navigation_climb", "navigation height source should include phase")
    assert(target.heading.angle == 0.75, "navigation should override heading")
    assert(target.heading.source == "navigation_climb", "navigation heading source should include phase")
end

local function checkModeTargetIsPure()
    local state = canonicalState()
    local machine = mode_state.new(state, config)
    local input = canonicalInputFromAxes({
        roll = 1.0,
        pitch = 0.0,
        climb = 0.0,
        heading = 0.0,
    })

    machine:update({
        input = input,
        state = state,
        navigationCommand = nil,
        dt = config.control.loop.dt,
    })

    local before = machine:terms()

    modeTarget(machine, {
        input = input,
        state = state,
        dt = config.control.loop.dt,
    })
    modeTarget(machine, {
        input = input,
        state = state,
        dt = config.control.loop.dt,
    })

    assertEquivalent("mode target purity", before, machine:terms())
end

local function checkNavigationHeadingWrap()
    local state = canonicalState()
    local machine = mode_state.new(state, config)

    state.navigation.heading.angle = 3.0
    machine.name = "navigation"
    machine.lastNavigation = {
        active = true,
        phase = "turn",
        target = {
            position = {
                x = 0.0,
                z = 0.0,
            },
            heading = -3.0,
        },
    }

    local target = modeTarget(machine, {
        input = input_protocol.defaultInput(),
        state = state,
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

local function checkEulerHeadingRateKinematics()
    local roll = 0.35
    local pitch = -0.42
    local heading = 0.80
    local headingRate = 0.70
    local frame = attitude_math.frameFromPose(roll, pitch, heading)
    local bodyRates = attitude_math.bodyRatesFromEulerRates(roll, pitch, {
        heading = headingRate,
    })
    local projectedHeadingRate = sensor_task.headingRateFromAngular(frame, bodyRates)

    assertClose("heading rate kinematics", projectedHeadingRate, headingRate)
end

local function checkManualEnterCapturesCurrentPose()
    local state = runtimeState()
    local machine = mode_state.new(state, config)
    local input = canonicalInputFromAxes({
        roll = 1.0,
        pitch = 0.0,
        climb = 0.0,
        heading = 0.0,
    })

    state.body.pose.roll = config.control.attitude.limit.roll * 2.0
    state.body.pose.pitch = -0.20
    machine.modes.manual.roll = -0.40
    machine.modes.manual.pitch = 0.40

    machine:update({
        input = input,
        state = state,
        navigationCommand = nil,
        dt = 0.0,
    })

    local terms = machine:terms().manual

    assertClose("manual enter roll capture clamp", terms.roll, config.control.attitude.limit.roll)
    assertClose("manual enter pitch capture", terms.pitch, -0.20)
end

local function checkManualHeadingFeedforwardUsesCurrentPose()
    local state = runtimeState()
    local machines = makeRuntimeMachines(state)
    local input = canonicalInputFromAxes({
        roll = 0.0,
        pitch = 0.0,
        climb = 0.0,
        heading = 1.0,
    })
    state.body.pose.roll = 0.25
    state.body.pose.pitch = -0.20
    machines.mode.modes.manual.roll = -0.10
    machines.mode.modes.manual.pitch = 0.30
    machines.mode.name = "manual"

    machines.mode:update({
        input = input,
        state = state,
        navigationCommand = nil,
        dt = config.control.loop.dt,
    })
    local target = modeTarget(machines.mode, {
        input = input,
        state = state,
        dt = config.control.loop.dt,
    })
    local expected = attitude_math.bodyRatesFromEulerRates(
        state.body.pose.roll,
        state.body.pose.pitch,
        {
            heading = config.control.heading.target_rate,
        }
    )
    local targetPoseRates = attitude_math.bodyRatesFromEulerRates(
        target.attitude.roll,
        target.attitude.pitch,
        {
            heading = config.control.heading.target_rate,
        }
    )

    assert(target.heading.source == "manual", "manual heading should use rate target source")
    assert(target.heading.active == false, "manual heading should not report locked heading")
    assertClose("manual heading target angle", target.heading.angle, state.navigation.heading.angle)
    assertClose("manual heading target rate", target.heading.rate, config.control.heading.target_rate)
    assertClose("manual heading target error", target.heading.error, 0.0)
    assertClose("manual heading roll feedforward", target.attitude.feedforward.angle.roll, expected.roll)
    assertClose("manual heading pitch feedforward", target.attitude.feedforward.angle.pitch, expected.pitch)
    assertClose("manual heading yaw feedforward", target.attitude.feedforward.angle.yaw, expected.yaw)
    assert(
        math.abs(target.attitude.feedforward.angle.roll - targetPoseRates.roll) > 1.0e-6,
        "manual heading feedforward should use current pose, not target attitude"
    )
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
    local terms = machine:terms()
    assert(terms.navigation.active == true, "navigation should remain active without a new command")
    assert(type(terms.navigation.target) == "table", "active navigation should keep a target every tick")
    assert(type(terms.navigation.target.position) == "table", "active navigation target should include position")
end

local function checkActiveNavigationActivateKeepsTarget()
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

    local activated = machine:update({
        input = input,
        state = state,
        navigationCommand = {
            action = "activate",
            waypoint = "home",
        },
        dt = config.control.loop.dt,
    })

    assert(activated.name == "navigation", "active activated navigation should remain active")
    local terms = machine:terms()
    assert(terms.navigation.active == true, "active activated navigation should remain active")
    assert(type(terms.navigation.target) == "table", "active activated navigation should keep a target")
    assert(type(terms.navigation.target.position) == "table", "active activated navigation target should include position")
end

local function checkActiveNavigationUpdateReceivesDt()
    local originalNavigation = package.loaded["navigation"]
    local originalModeState = package.loaded["state.mode_state"]
    local originalModesNavigation = package.loaded["modes.navigation"]
    local observedDt = nil
    local fakeNavigator = nil

    local ok, err = pcall(function()
        package.loaded["state.mode_state"] = nil
        package.loaded["modes.navigation"] = nil
        package.loaded["navigation"] = {
            new = function()
                fakeNavigator = {
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
                    cancel = function()
                        return {
                            active = false,
                        }
                    end,
                }

                return fakeNavigator
            end,
        }

        local fakeModeState = require("state.mode_state")
        local machine = fakeModeState.new(runtimeState(), config)
        machine.name = "navigation"
        machine:update({
            input = input_protocol.defaultInput(),
            state = runtimeState(),
            navigationCommand = nil,
            dt = 0.123,
        })
    end)

    package.loaded["navigation"] = originalNavigation
    package.loaded["modes.navigation"] = originalModesNavigation
    package.loaded["state.mode_state"] = originalModeState

    assert(ok, err)
    assert(math.abs(observedDt - 0.123) < 1.0e-9, "active navigation update should receive real dt")
end

local function checkNavigationEnterUpdatesOnce()
    local originalNavigation = package.loaded["navigation"]
    local originalModeState = package.loaded["state.mode_state"]
    local originalModesNavigation = package.loaded["modes.navigation"]
    local updateCount = 0
    local observedDt = nil

    local ok, err = pcall(function()
        package.loaded["state.mode_state"] = nil
        package.loaded["modes.navigation"] = nil
        package.loaded["navigation"] = {
            new = function()
                return {
                    update = function(_, _, dt)
                        updateCount = updateCount + 1
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
                            active = false,
                            phase = "idle",
                            target = nil,
                        }
                    end,
                    command = function()
                        return {
                            active = true,
                            phase = "climb",
                            target = nil,
                        }
                    end,
                    cancel = function()
                        return {
                            active = false,
                        }
                    end,
                }
            end,
        }

        local fakeModeState = require("state.mode_state")
        local machine = fakeModeState.new(runtimeState(), config)

        machine:update({
            input = input_protocol.defaultInput(),
            state = runtimeState(),
            navigationCommand = {
                action = "activate",
                waypoint = "home",
            },
            dt = 0.234,
        })
    end)

    package.loaded["navigation"] = originalNavigation
    package.loaded["modes.navigation"] = originalModesNavigation
    package.loaded["state.mode_state"] = originalModeState

    assert(ok, err)
    assert(updateCount == 1, "navigation enter should not update before lifecycle update")
    assert(math.abs(observedDt - 0.234) < 1.0e-9, "navigation lifecycle update should receive real dt")
end

local function checkCruiseToggleOneShot()
    local state = canonicalState()
    local machine = mode_state.new(state, config)
    local input = input_protocol.defaultInput()

    state.world.velocity = vector.new(3.0, 0.0, -1.0)
    input.event.cruiseToggle = true
    machine.name = "manual"

    machine:update({
        input = input,
        state = state,
        navigationCommand = nil,
        dt = config.control.loop.dt,
    })
    local first = machine:terms()

    state.world.velocity = vector.new(9.0, 0.0, 9.0)

    machine:update({
        input = input,
        state = state,
        navigationCommand = nil,
        dt = config.control.loop.dt,
    })
    local second = machine:terms()

    assert(first.cruise.velocity.x == 3.0, "first cruise toggle should capture velocity")
    assert(first.cruise.velocity.y == 0.0, "cruise toggle should capture horizontal velocity")
    assert(type(first.cruise.velocity.length) == "function", "cruise velocity should be runtime vector")
    assert(first.cruise.height == 80.0, "cruise should freeze entry height")
    assert(first.cruise.heading == 0.0, "cruise should freeze entry heading")
    assert(second.cruise.velocity.x == 3.0, "held cruise toggle should not recapture velocity")
end

local function checkCruiseRequiresManualMode()
    local state = canonicalState()
    local machine = mode_state.new(state, config)
    local input = input_protocol.defaultInput()

    state.world.velocity = vector.new(3.0, 0.0, -1.0)
    input.event.cruiseToggle = true

    local mode = machine:update({
        input = input,
        state = state,
        navigationCommand = nil,
        dt = config.control.loop.dt,
    })

    assert(mode.name == "position_hold", "cruise toggle outside manual should stay in position_hold")
    assert(machine:terms().cruise == nil, "cruise toggle outside manual should not capture velocity")
end

local function checkCruiseFreezesAxes()
    local state = canonicalState()
    local machine = mode_state.new(state, config)
    local input = input_protocol.defaultInput()

    state.world.velocity = vector.new(3.0, 0.0, -1.0)
    machine.name = "manual"
    input.event.cruiseToggle = true

    machine:update({
        input = input,
        state = state,
        navigationCommand = nil,
        dt = config.control.loop.dt,
    })

    input = input_protocol.defaultInput()
    state.world.velocity = vector.new(9.0, 0.0, 9.0)
    state.body.pose.height = 90.0
    state.navigation.heading.angle = 1.0

    machine:update({
        input = input,
        state = state,
        navigationCommand = nil,
        dt = config.control.loop.dt,
    })

    local target = modeTarget(machine, {
        input = input,
        state = state,
        dt = config.control.loop.dt,
    })

    assert(target.world.velocity.x == 3.0, "cruise should keep entry velocity")
    assert(target.world.velocity.z == -1.0, "cruise should keep entry velocity z")
    assert(target.vertical.height == 80.0, "cruise should keep entry height")
    assert(target.vertical.error == -10.0, "cruise height error should use current height")
    assert(target.heading.angle == 0.0, "cruise should keep entry heading")
    assert(math.abs(target.heading.error + 1.0) < 1.0e-9, "cruise heading error should use current heading")
end

local function checkNavigationCommandIgnoresManualOverride()
    local state = canonicalState()
    local machine = mode_state.new(state, config)
    local input = canonicalInputFromAxes({
        roll = 1.0,
        pitch = 0.0,
        climb = 0.0,
        heading = 0.0,
    })

    local mode = machine:update({
        input = input,
        state = state,
        navigationCommand = {
            action = "activate",
            waypoint = "home",
        },
        dt = config.control.loop.dt,
    })

    assert(mode.name == "manual", "manual override should win over navigation command")
    assert(machine:terms().navigation.active == false, "manual override should not activate navigation")
end

local function checkClimbCancelsNavigationToHold()
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

    input = canonicalInputFromAxes({
        roll = 0.0,
        pitch = 0.0,
        climb = 1.0,
        heading = 0.0,
    })

    local mode = machine:update({
        input = input,
        state = state,
        navigationCommand = nil,
        dt = config.control.loop.dt,
    })

    assert(mode.name == "position_hold", "climb override should cancel navigation into position_hold")
    assert(mode.transition.navigationExited == true, "climb override should report navigation exit")
    assert(machine:terms().navigation.active == false, "climb override should leave navigation inactive")
end

local function checkNavigationManualOverrideDestinations()
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

    local headingMode = machine:update({
        input = canonicalInputFromAxes({
            roll = 0.0,
            pitch = 0.0,
            climb = 0.0,
            heading = 1.0,
        }),
        state = state,
        navigationCommand = nil,
        dt = config.control.loop.dt,
    })

    assert(headingMode.name == "manual", "heading override should cancel navigation into manual")
    assert(headingMode.transition.navigationExited == true, "heading override should report navigation exit")

    machine = mode_state.new(state, config)
    machine:update({
        input = input,
        state = state,
        navigationCommand = {
            action = "activate",
            waypoint = "home",
        },
        dt = config.control.loop.dt,
    })

    local lateralMode = machine:update({
        input = canonicalInputFromAxes({
            roll = 1.0,
            pitch = 0.0,
            climb = 0.0,
            heading = 0.0,
        }),
        state = state,
        navigationCommand = nil,
        dt = config.control.loop.dt,
    })

    assert(lateralMode.name == "manual", "lateral override should cancel navigation into manual")
    assert(lateralMode.transition.navigationExited == true, "lateral override should report navigation exit")
end

local function checkNavigationExitRelockTarget()
    local state = canonicalState()
    local modes = mode_state.new(state, config)
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
            action = "cancel",
        },
        dt = config.control.loop.dt,
    })

    assert(mode.transition.navigationExited == true, "mode state should report navigation exit edge")

    local target = modeTarget(modes, {
        input = input,
        state = state,
        dt = config.control.loop.dt,
    })

    assert(mode.name == "position_hold", "navigation cancel exit should return to position_hold")
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
        navigation = {
            active = false,
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
        angle = 0.0,
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
                        angle = pidTerms(),
                        rate = pidTerms(),
                    },
                    pitch = {
                        angle = pidTerms(),
                        rate = pidTerms(),
                    },
                    yaw = {
                        angle = pidTerms(),
                        rate = pidTerms(),
                    },
                },
                feedforward = {
                    angle = {
                        roll = 0.0,
                        pitch = 0.0,
                        yaw = 0.0,
                    },
                    rate = {
                        roll = 0.0,
                        pitch = 0.0,
                        yaw = 0.0,
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
                finalCommands = {
                    collective = 1.0,
                    roll = 0.0,
                    pitch = 0.0,
                    yaw = 0.0,
                },
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

    local sawAnglePid = false
    local sawVerticalError = false

    for _, write in ipairs(mon.writes) do
        if write.text:find("RANG", 1, true) ~= nil then
            sawAnglePid = true
        end
        if write.text:find("%+17%.0", 1, false) ~= nil then
            sawVerticalError = true
        end
    end

    assert(sawAnglePid, "attitude page should show angle PID rows")
    assert(sawVerticalError, "navigation page vertical error should use target height")
end

local function checkUiNavigationCommands()
    local mon = fakeMonitor(80, 30)
    local shared = {
        telemetry = canonicalTelemetry(),
        telemetryTime = os.clock(),
        telemetrySender = 1,
        inputSeq = 1,
        monitorPage = "nav",
    }

    monitor_view.draw(mon, shared)

    local row = shared.monitorTouch.navRows[1]

    assert(type(row) == "table", "navigation page should expose touch row")
    assert(monitor_view.handleTouch(mon, shared, row.x1, row.y), "inactive waypoint touch should be handled")
    assert(shared.pendingNavigationCommand.action == "activate", "inactive waypoint touch should activate")
    assert(shared.pendingNavigationCommand.waypoint == "home", "inactive waypoint touch should target waypoint")

    shared.pendingNavigationCommand = nil
    shared.telemetry.navigation.active = true
    shared.telemetry.navigation.waypoint = shared.telemetry.navigation.selected
    monitor_view.draw(mon, shared)

    row = shared.monitorTouch.navRows[1]

    assert(monitor_view.handleTouch(mon, shared, row.x1, row.y), "active waypoint touch should be handled")
    assert(shared.pendingNavigationCommand.action == "cancel", "active waypoint touch should cancel")
    assert(shared.pendingNavigationCommand.waypoint == nil, "cancel command should not include waypoint")
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
    state.body.pose.roll = 0.25
    local machines = makeRuntimeMachines(state)
    local input = canonicalInputFromAxes({
        roll = 1.0,
        pitch = 0.0,
        climb = 0.0,
        heading = 0.0,
    })
    machines.mode.name = "manual"
    machines.mode:update({
        input = input,
        state = state,
        navigationCommand = nil,
        dt = config.control.loop.dt,
    })
    local target = modeTarget(machines.mode, {
        input = input,
        state = state,
        dt = config.control.loop.dt,
    })
    local modeTerms = machines.mode:terms()
    local command, oldDetails = machines.controller:update({
        state = state,
        target = target,
        reset = modeTerms.reset,
        dt = config.control.loop.dt,
    })
    local terms = machines.controller:terms()

    assert(oldDetails == nil, "controller update should return command only")
    assert(type(command.collective) == "number", "controller command should contain collective")
    assert(type(terms.horizontal) == "table", "controller terms should include horizontal")
    assert(type(terms.vertical) == "table", "controller terms should include vertical")
    assert(type(terms.attitude) == "table", "controller terms should include attitude")
    assert(type(terms.allocation) == "table", "controller terms should include allocation")
    assert(terms.horizontal == machines.controller.horizontal:terms(), "controller should compose horizontal terms from horizontal")
    assert(terms.vertical == machines.controller.vertical:terms(), "controller should compose vertical terms from vertical")
    assert(terms.attitude == machines.controller.attitude:terms(), "controller should compose attitude terms from attitude")
    assert(terms.allocation == machines.controller.allocation:terms(), "controller should compose allocation terms from allocation")
    assert(terms.output == nil, "controller terms should not duplicate final command under output")
    assert(type(terms.allocation.rawCommands) == "table", "allocation terms should include raw commands")
    assert(type(terms.allocation.allocatedCommands) == "table", "allocation terms should include allocated commands")
    assert(type(terms.allocation.finalCommands) == "table", "allocation terms should include final commands")
    assert(type(terms.allocation.debug) == "table", "allocation terms should include allocator debug")
    assert(math.abs(terms.allocation.finalCommands.collective - command.collective) < 1.0e-6, "final collective should match top-level command")
    assert(math.abs(terms.allocation.finalCommands.roll - command.roll) < 1.0e-6, "final roll should match top-level command")
    assert(math.abs(terms.allocation.finalCommands.pitch - command.pitch) < 1.0e-6, "final pitch should match top-level command")
    assert(math.abs(terms.allocation.finalCommands.yaw - command.yaw) < 1.0e-6, "final yaw should match top-level command")
    assert(type(terms.horizontal.terms.position.forward.output) == "number", "horizontal should own position pid terms")
    assert(type(terms.vertical.terms.height.output) == "number", "vertical should own height pid terms")
    assert(config.control.attitude.time_constant == nil, "attitude time_constant should be removed")
    assertClose("heading target rate", config.control.heading.target_rate, math.rad(60))
    assert(type(config.control.pid.attitude.roll.angle) == "table", "roll angle pid config should exist")
    assert(type(config.control.pid.attitude.pitch.angle) == "table", "pitch angle pid config should exist")
    assert(type(config.control.pid.attitude.yaw.angle) == "table", "yaw angle pid config should exist")
    assertClose("roll angle kp", config.control.pid.attitude.roll.angle.kp, 1.80)
    assertClose("roll angle ki", config.control.pid.attitude.roll.angle.ki, 0.18)
    assertClose("roll angle kd", config.control.pid.attitude.roll.angle.kd, 0.05)
    assertClose("pitch angle kp", config.control.pid.attitude.pitch.angle.kp, 1.25)
    assertClose("pitch angle ki", config.control.pid.attitude.pitch.angle.ki, 0.20)
    assertClose("pitch angle kd", config.control.pid.attitude.pitch.angle.kd, 0.05)
    assertClose("yaw angle kp", config.control.pid.attitude.yaw.angle.kp, 0.85)
    assertClose("yaw angle ki", config.control.pid.attitude.yaw.angle.ki, 0.0)
    assertClose("yaw angle kd", config.control.pid.attitude.yaw.angle.kd, 0.25)
    assert(type(terms.attitude.terms.roll.angle.output) == "number", "attitude should own roll angle pid terms")
    assert(type(terms.attitude.terms.pitch.angle.output) == "number", "attitude should own pitch angle pid terms")
    assert(type(terms.attitude.terms.yaw.angle.output) == "number", "attitude should own yaw angle pid terms")
    assert(terms.attitude.current.roll.angle == 0.0, "roll angle pid current should be zero quaternion-error reference")
    assert(terms.attitude.current.roll.angle ~= state.body.pose.roll, "roll angle pid current should not be body pose roll")
    assert(math.abs(terms.attitude.target.roll.rate - terms.attitude.terms.roll.angle.output) < 1.0e-6, "roll rate target should come from angle pid output")
    assert(type(terms.attitude.terms.roll.rate.output) == "number", "attitude should own rate pid terms")
    assert(math.abs(terms.allocation.rawCommands.roll - terms.attitude.terms.roll.rate.output) < 1.0e-6, "rate pid output should match raw roll command")
end

local function checkControllerResetInput()
    local state = runtimeState()
    local controller = Controller.new(config.control)
    local resetCalled = false

    controller.horizontal.reset = function()
        resetCalled = true
    end

    controller:update({
        state = state,
        target = {
            source = "position_hold",
            attitude = {
                roll = nil,
                pitch = nil,
                feedforward = {
                    angle = {
                        roll = 0.0,
                        pitch = 0.0,
                        yaw = 0.0,
                    },
                    rate = {
                        roll = 0.0,
                        pitch = 0.0,
                        yaw = 0.0,
                    },
                },
            },
            world = {
                position = vector.new(-213.0, 0.0, 304.0),
                velocity = nil,
                acceleration = nil,
            },
            vertical = {
                height = 80.0,
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
        },
        reset = {
            horizontal = true,
        },
        dt = config.control.loop.dt,
    })

    assert(resetCalled, "controller should read horizontal reset from lifecycle input")
end

local function checkAttitudeExternalFeedforward()
    local state = runtimeState()
    local machines = makeRuntimeMachines(state)
    local input = canonicalInputFromAxes({
        roll = 0.0,
        pitch = 0.0,
        climb = 0.0,
        heading = 0.0,
    })
    machines.mode.name = "manual"
    machines.mode:update({
        input = input,
        state = state,
        navigationCommand = nil,
        dt = config.control.loop.dt,
    })
    local target = modeTarget(machines.mode, {
        input = input,
        state = state,
        dt = config.control.loop.dt,
    })

    target.attitude.feedforward.angle.roll = 0.25
    target.attitude.feedforward.angle.pitch = -0.50
    target.attitude.feedforward.angle.yaw = 0.75
    target.attitude.feedforward.rate.roll = 0.125
    target.attitude.feedforward.rate.pitch = -0.250
    target.attitude.feedforward.rate.yaw = 0.375

    machines.controller:update({
        state = state,
        target = target,
        reset = machines.mode:terms().reset,
        dt = config.control.loop.dt,
    })

    local terms = machines.controller:terms()

    assertClose("roll angle feedforward term", terms.attitude.feedforward.angle.roll, 0.25)
    assertClose("pitch angle feedforward term", terms.attitude.feedforward.angle.pitch, -0.50)
    assertClose("yaw angle feedforward term", terms.attitude.feedforward.angle.yaw, 0.75)
    assertClose("roll rate feedforward term", terms.attitude.feedforward.rate.roll, 0.125)
    assertClose("pitch rate feedforward term", terms.attitude.feedforward.rate.pitch, -0.250)
    assertClose("yaw rate feedforward term", terms.attitude.feedforward.rate.yaw, 0.375)
    assertClose("roll rate target feedforward", terms.attitude.target.roll.rate, terms.attitude.terms.roll.angle.output + 0.25)
    assertClose("pitch rate target feedforward", terms.attitude.target.pitch.rate, terms.attitude.terms.pitch.angle.output - 0.50)
    assertClose("yaw rate target feedforward", terms.attitude.target.yaw.rate, terms.attitude.terms.yaw.angle.output + 0.75)
    assertClose("roll command feedforward", terms.allocation.rawCommands.roll, terms.attitude.terms.roll.rate.output + 0.125)
    assertClose("pitch command feedforward", terms.allocation.rawCommands.pitch, terms.attitude.terms.pitch.rate.output - 0.250)
    assertClose("yaw command feedforward", terms.allocation.rawCommands.yaw, terms.attitude.terms.yaw.rate.output + 0.375)
end

checkFrozenBaseline()
checkProtocolDecode()
checkFlightState()
checkModeUpdateExposesStatusOnly()
checkModeTermsSnapshotsAreCopied()
checkModeTargetNavigationOverride()
checkModeTargetIsPure()
checkNavigationHeadingWrap()
checkNavigationVelocityFrame()
checkEulerHeadingRateKinematics()
checkManualEnterCapturesCurrentPose()
checkManualHeadingFeedforwardUsesCurrentPose()
checkActiveNavigationKeepsTarget()
checkActiveNavigationActivateKeepsTarget()
checkActiveNavigationUpdateReceivesDt()
checkNavigationEnterUpdatesOnce()
checkCruiseToggleOneShot()
checkCruiseRequiresManualMode()
checkCruiseFreezesAxes()
checkNavigationCommandIgnoresManualOverride()
checkNavigationManualOverrideDestinations()
checkClimbCancelsNavigationToHold()
checkNavigationExitRelockTarget()
checkTelemetryPreservesConsumedCruiseEvent()
checkUiTelemetryBoundary()
checkUiNavigationCommands()
checkMixerFormula()
checkControllerTerms()
checkControllerResetInput()
checkAttitudeExternalFeedforward()

assertOldRuntimeModuleRemoved("control_task")
assertOldRuntimeModuleRemoved("input_task")
assertOldRuntimeModuleRemoved("data_task")
assertOldRuntimeModuleRemoved("rotor")
assertOldRuntimeModuleRemoved("target_state")
assertOldRuntimeModuleRemoved("trajectory")

print("control fixtures ok")
