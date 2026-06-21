local env = require("tools.test_env")
env.install()

local baseline = require("tools.fixtures.control_baseline")
local config = require("config")
local flight_state = require("state.flight_state")
local input_protocol = require("protocol.input")
local mixer = require("hardware.mixer")
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
checkMixerFormula()

assertOldRuntimeModuleRemoved("control_task")
assertOldRuntimeModuleRemoved("input_task")
assertOldRuntimeModuleRemoved("data_task")
assertOldRuntimeModuleRemoved("rotor")
assertOldRuntimeModuleRemoved("target_state")

print("control fixtures ok")
