local env = require("tools.test_env")
env.install()

local baseline = require("tools.fixtures.control_baseline")
local config = require("config")
local flight_state = require("state.flight_state")
local heading_lock = require("state.heading_lock")
local height_lock = require("state.height_lock")
local input_protocol = require("protocol.input")
local mode_state = require("state.mode_state")
local mixer = require("hardware.mixer")
local sensor_task = require("tasks.sensor_task")
local telemetry_terms = require("telemetry.terms")
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

local function assertOldRuntimeModuleRemoved(name)
    local ok = pcall(require, name)

    assert(not ok, "old root runtime module should not be requireable: " .. name)
end

local function checkProtocolDecode()
    local input = input_protocol.decode({
        controls = {
            roll = 2.0,
            pitch = -2.0,
            climb = 0.25,
            heading = -0.5,
        },
        event = {
            cruiseLock = true,
            navigation = {
                action = "activate",
                waypoint = "home",
            },
        },
    })

    assert(input.controls == nil, "decoded input must not expose old controls table")
    assert(input.manual.attitude.roll == 1.0, "roll input should clamp")
    assert(input.manual.attitude.pitch == -1.0, "pitch input should clamp")
    assert(input.manual.velocity.up == 0.25, "climb input should become manual velocity up")
    assert(input.manual.heading.rate == -0.5, "heading input should become heading rate")
    assert(input.event.cruiseToggle == true, "cruise event should decode")
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

local function checkTelemetryPreservesConsumedCruiseEvent()
    local telemetry = telemetry_terms.running({
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
        details = {
            output = {},
            current = {},
            error = {},
            terms = {},
            pid = {},
            positionHold = {},
        },
        rotor = {},
    })

    assert(telemetry.input.event.cruiseToggle == true, "telemetry should preserve consumed cruise event")
    assert(type(telemetry.state.body.angular.velocity) == "table", "telemetry should expose angular velocity")
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

checkProtocolDecode()
checkFlightState()
checkTrajectoryNavigationOverride()
checkNavigationHeadingWrap()
checkNavigationVelocityFrame()
checkCruiseToggleOneShot()
checkNavigationExitRelockTargets()
checkTelemetryPreservesConsumedCruiseEvent()
checkMixerFormula()

assertOldRuntimeModuleRemoved("control_task")
assertOldRuntimeModuleRemoved("input_task")
assertOldRuntimeModuleRemoved("data_task")
assertOldRuntimeModuleRemoved("rotor")
assertOldRuntimeModuleRemoved("target_state")

print("control fixtures ok")
