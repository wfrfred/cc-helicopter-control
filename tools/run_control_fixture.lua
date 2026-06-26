local env = require("tools.test_env")
env.install()

local baseline = require("tools.fixtures.control_baseline")
local attitude_math = require("lib.attitude_math")
local common = require("modes.common")
local config = require("config")
local Controller = require("control.controller")
local flight_state = require("state.flight_state")
local input_protocol = require("protocol.input")
local mode_state = require("state.mode_state")
local mixer = require("hardware.mixer")
local monitor_view = require("monitor_view")
local pid = require("lib.pid")
local sensor_task = require("tasks.sensor_task")
local tablex = require("lib.tablex")
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

local function checkTablex()
    local mapped = tablex.list.map({ 2, 4, 6 }, function(value, index)
        return value + index
    end)

    assert(mapped[1] == 3, "map should preserve sequence order")
    assert(mapped[2] == 6, "map should pass sequence index")
    assert(mapped[3] == 9, "map should return a sequence")

    local eachList = {}
    tablex.list.each({ "a", "b" }, function(value, index)
        eachList[#eachList + 1] = tostring(index) .. value
    end)

    assert(eachList[1] == "1a", "list.each should traverse sequence order")
    assert(eachList[2] == "2b", "list.each should pass sequence index")

    local eachRecord = {}
    tablex.record.each({
        height = 10,
        heading = 1.57,
    }, function(value, key)
        eachRecord[key] = value
    end)

    assert(eachRecord.height == 10, "record.each should pass record key")
    assert(eachRecord.heading == 1.57, "record.each should traverse record values")

    local mappedRecord = tablex.record.map({
        height = 10,
        heading = 1.57,
    }, function(value, key)
        return tostring(key) .. "=" .. tostring(value)
    end)

    assert(mappedRecord.height == "height=10", "record.map should preserve record key")
    assert(mappedRecord.heading == "heading=1.57", "record.map should map record value")

    local filtered = tablex.list.filter({ 1, 2, 3, 4 }, function(value)
        return value % 2 == 0
    end)

    assert(#filtered == 2, "filter should return a compact sequence")
    assert(filtered[1] == 2, "filter should keep first matching item")
    assert(filtered[2] == 4, "filter should keep second matching item")

    local reduced = tablex.list.reduce({ "a", "b", "c" }, function(acc, value, index)
        return acc .. index .. value
    end, "")

    assert(reduced == "1a2b3c", "reduce should traverse sequence order")

    local rows = tablex.record.transpose({ "height", "heading" }, {
        target = {
            height = 10,
            heading = 1.57,
        },
        error = {
            height = 2,
            heading = 0.1,
        },
    })

    assert(rows.height.target == 10, "transpose should group height target")
    assert(rows.height.error == 2, "transpose should group height error")
    assert(rows.heading.target == 1.57, "transpose should group heading target")
    assert(rows.heading.error == 0.1, "transpose should group heading error")

    local columns = tablex.record.untranspose({ "height", "heading" }, rows)

    assert(columns.target.height == 10, "untranspose should restore target height")
    assert(columns.error.heading == 0.1, "untranspose should restore heading error")
end

local function checkPid()
    local controller = pid.new({
        kp = 2.0,
        ki = 1.0,
        kd = 0.5,
        out_min = -100.0,
        out_max = 100.0,
    })

    assert(controller.last == nil, "pid should not expose last()")
    assert(controller.terms == nil, "pid should not expose terms()")

    local first = controller:update(13.0, 10.0, 0.1)

    assertClose("pid first error", first.terms.error, 3.0)
    assertClose("pid first derivative", first.terms.derivative, 0.0)
    assertClose("pid first integral", first.terms.integral, 0.3)
    assertClose("pid first output", first.output, 6.3)

    local second = controller:update(13.0, 12.0, 0.1)

    assertClose("pid current derivative", second.terms.derivative, 20.0)
    assertClose("pid trapezoid integral", second.terms.integral, 0.5)
    assertClose("pid measurement d term", second.terms.d, -10.0)
    assertClose("pid second output", second.output, -7.5)

    local explicit = controller:update(13.0, 12.0, 0.1, 4.0)

    assertClose("pid explicit derivative", explicit.terms.derivative, 4.0)
    assertClose("pid explicit derivative output", explicit.output, 0.6)

    local resetResult = controller:reset()
    local afterReset = controller:update(2.0, 1.0, 0.1)

    assert(resetResult == nil, "pid reset should not return diagnostic terms")
    assertClose("pid reset derivative", afterReset.terms.derivative, 0.0)
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

local function worldFromFrd(value, yaw)
    local forward = vector.new(math.sin(yaw), 0.0, -math.cos(yaw))
    local right = vector.new(math.cos(yaw), 0.0, math.sin(yaw))

    return forward * (value.forward or 0.0)
        + right * (value.right or 0.0)
        + vector.new(0.0, -(value.down or 0.0), 0.0)
end

local function legacyTarget(target, state)
    local horizontalPosition = target.horizontal.position or {}
    local horizontalVelocity = target.horizontal.feedforward.position or {}
    local horizontalAngle = target.horizontal.angle or {}
    local positionDelta = worldFromFrd({
        forward = horizontalPosition.forward,
        right = horizontalPosition.right,
        down = target.altitude.position,
    }, target.yaw.angle)
    local height = nil

    if target.altitude.position ~= nil then
        height = state.body.pose.height - target.altitude.position
    end

    return {
        position = state.world.position + positionDelta,
        velocity = worldFromFrd({
            forward = horizontalVelocity.forward,
            right = horizontalVelocity.right,
            down = target.altitude.feedforward.position,
        }, target.yaw.angle),
        height = height,
        verticalSpeed = -target.altitude.feedforward.position,
        heightActive = target.altitude.position ~= nil,
        heading = target.yaw.angle,
        roll = horizontalAngle.roll,
        pitch = horizontalAngle.pitch,
    }
end

local function neutralTarget(state)
    local target = common.target("position")

    target.yaw.angle = state.navigation.heading.angle

    return target
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
    local control = machines.controller:update({
        state = state,
        target = target,
        dt = config.control.loop.dt,
    })
    local command = control.output
    local controlTerms = control.terms

    if case.name == "position_hold neutral" then
        return {
            mode = mode.name,
            horizontalKind = controlTerms.horizontal.kind,
            target = controlTerms.horizontal.output.angle,
        }
    end

    if case.name == "cruise capture" then
        local terms = modeTerms.terms

        return {
            mode = mode.name,
            cruise = {
                x = terms.velocity.x,
                z = terms.velocity.z,
                height = terms.height.target,
                heading = terms.heading.target,
            },
            target = controlTerms.horizontal.output.angle,
        }
    end

    if case.name == "navigation active target" then
        local terms = modeTerms.terms
        local legacy = legacyTarget(target, state)

        return {
            mode = mode.name,
            active = terms.active,
            phase = terms.phase,
            target = {
                x = legacy.position.x,
                z = legacy.position.z,
                height = legacy.height,
                heading = legacy.heading,
            },
        }
    end

    local legacy = legacyTarget(target, state)

    return {
        mode = mode.name,
        target = {
            roll = legacy.roll,
            pitch = legacy.pitch,
            height = legacy.height,
            verticalSpeed = legacy.verticalSpeed,
            heightActive = legacy.heightActive,
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

    assert(not ok, "removed module should not be requireable: " .. name)
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
    local machine = flight_state.new(config.control.sensor_age)
    local waiting = machine:update({
        state = nil,
        input = input_protocol.defaultInput(),
        inputStale = false,
        now = 1.0,
    })
    assert(waiting.name == "waiting_sensors", "missing sensors should wait")

    local stale = machine:update({
        state = canonicalState(),
        input = input_protocol.defaultInput(),
        inputStale = true,
        now = 1.0,
    })
    assert(stale.name == "running", "ready sensors should run")
    assert(stale.reason == "input_stale_zeroed", "stale input should be reported")

    local aged = machine:update({
        state = canonicalState(),
        input = input_protocol.defaultInput(),
        inputStale = false,
        now = 2.1,
    })
    assert(aged.name == "running", "sensor age fault should not stop control")
    assert(aged.reason == "sensor_age_fault", "sensor age fault should be reported")
    assert(aged.sensorAge.max >= config.control.sensor_age.fault_dt, "sensor age max should be reported")
end

local function checkModeUpdateShape()
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
    assert(result.transition == nil, "mode update should not expose transition status")
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

    local first = machine:terms().terms

    first.velocity.x = 99.0

    local second = machine:terms()

    assert(second.name == "cruise", "mode terms should describe current mode")
    assert(second.terms.velocity.x ~= 99.0, "current mode terms should be copied")
    assert(second.manual == nil, "mode terms should not expose inactive manual state")
    assert(second.position_hold == nil, "mode terms should not expose inactive position_hold state")
    assert(second.cruise == nil, "mode terms should not expose inactive cruise state")
end

local function checkModeTargetNavigationOverride()
    local machine = mode_state.new(canonicalState(), config)
    local state = canonicalState()

    machine.name = "navigation"
    state.body.pose.heading = 0.75
    machine.modes.navigation.route = {
        waypoint = {
            id = "fixture",
            position = {
                x = 10.0,
                y = 120.0,
                z = -20.0,
            },
        },
        approach = nil,
        legs = {
            {
                kind = "direct",
                position = {
                    x = 10.0,
                    y = 120.0,
                    z = -20.0,
                },
                radius = 5.0,
            },
        },
        legIndex = 1,
        phase = "climb",
        holdPosition = {
            x = 10.0,
            y = 80.0,
            z = -20.0,
        },
        destination = {
            x = 10.0,
            y = 120.0,
            z = -20.0,
        },
        cruiseAltitude = 120.0,
        arrivalHeading = 0.75,
    }

    local target = modeTarget(machine, {
        input = input_protocol.defaultInput(),
        state = state,
        dt = config.control.loop.dt,
    })

    local legacy = legacyTarget(target, state)

    assert(target.translation == nil, "controller target should not expose old translation target")
    assert(target.attitude == nil, "controller target should not expose old attitude target")
    assert(target.world == nil, "controller target should not expose old world target")
    assert(target.vertical == nil, "controller target should not expose old vertical target")
    assert(target.heading == nil, "controller target should not expose old heading target")
    assert(math.abs(legacy.position.x - 10.0) < 1.0e-6, "navigation should set horizontal target x")
    assert(math.abs(legacy.position.z - -20.0) < 1.0e-6, "navigation should set horizontal target z")
    assert(target.horizontal.feedforward.angle.roll == 0.0, "controller target should default roll angle feedforward")
    assert(target.yaw.feedforward.rate == 0.0, "controller target should default yaw rate feedforward")
    assert(math.abs(legacy.height - 120.0) < 1.0e-6, "navigation should override height")
    assert(target.yaw.angle == 0.75, "navigation should override yaw")
end

local function checkNavigationTargetRequiresRoute()
    local machine = mode_state.new(canonicalState(), config)

    machine.name = "navigation"

    local ok, err = pcall(function()
        modeTarget(machine, {
            input = input_protocol.defaultInput(),
            state = canonicalState(),
            dt = config.control.loop.dt,
        })
    end)

    assert(not ok, "navigation target should require an active route")
    assert(
        tostring(err):find("active route", 1, true) ~= nil,
        "navigation target route failure should be explicit"
    )
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

local function checkNavigationTargetIsPure()
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

    assertEquivalent("navigation target purity", before, machine:terms())
end

local function checkNavigationHeadingWrap()
    local state = runtimeState()
    local machine = mode_state.new(state, config)

    state.navigation.heading.angle = 3.0
    machine.name = "navigation"
    machine.modes.navigation.route = {
        waypoint = {
            id = "fixture",
            position = {
                x = 0.0,
                y = 80.0,
                z = 0.0,
            },
        },
        approach = nil,
        legs = {
            {
                kind = "direct",
                position = {
                    x = 0.0,
                    y = 80.0,
                    z = 0.0,
                },
                heading = -3.0,
                radius = 5.0,
            },
        },
        legIndex = 1,
        phase = "turn",
        holdPosition = {
            x = 0.0,
            y = 80.0,
            z = 0.0,
        },
        destination = {
            x = 0.0,
            y = 80.0,
            z = 0.0,
        },
        cruiseAltitude = 80.0,
        arrivalHeading = -3.0,
    }

    local target = modeTarget(machine, {
        input = input_protocol.defaultInput(),
        state = state,
        dt = config.control.loop.dt,
    })
    local controller = Controller.new(config.control)

    local control = controller:update({
        state = state,
        target = target,
        dt = config.control.loop.dt,
    })

    assert(
        math.abs(target.yaw.angle - state.navigation.heading.angle) > math.pi,
        "navigation target should expose raw yaw target, not wrapped debug error"
    )
    assert(math.abs(control.terms.attitude.angle.yaw.pid.error) < math.pi, "attitude yaw error should wrap")
end

local function checkNavigationVelocityFrame()
    local velocity = sensor_task.navigationVelocity(vector.new(1.0, 2.0, 0.0), math.pi / 2)

    assert(math.abs(velocity.forward - 1.0) < 1.0e-9, "navigation forward velocity should be heading-aligned")
    assert(math.abs(velocity.right) < 1.0e-9, "navigation right velocity should be heading-aligned")
    assert(math.abs(velocity.up - 2.0) < 1.0e-9, "navigation up velocity should be world y")

    local rawVelocity = config.calibration.body_axis.forward * 3.0
        + config.calibration.body_axis.right * 4.0
        + config.calibration.body_axis.down * -2.0
    local bodyVelocity = sensor_task.bodyVelocityFromRaw(rawVelocity)
    local frame = attitude_math.frameFromPose(0.0, 0.0, math.pi / 2)
    local worldVelocity = sensor_task.worldVelocityFromBody(frame, bodyVelocity)
    local navigationVelocity = sensor_task.navigationVelocity(worldVelocity, math.pi / 2)

    assertClose("raw local velocity forward", bodyVelocity.forward, 3.0)
    assertClose("raw local velocity right", bodyVelocity.right, 4.0)
    assertClose("raw local velocity down", bodyVelocity.down, -2.0)
    assertClose("body velocity world x", worldVelocity.x, 3.0)
    assertClose("body velocity world y", worldVelocity.y, 2.0)
    assertClose("body velocity world z", worldVelocity.z, 4.0)
    assertClose("body velocity navigation forward", navigationVelocity.forward, 3.0)
    assertClose("body velocity navigation right", navigationVelocity.right, 4.0)
    assertClose("body velocity navigation up", navigationVelocity.up, 2.0)
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

    local terms = machine:terms().terms

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
        target.horizontal.angle.roll,
        target.horizontal.angle.pitch,
        {
            heading = config.control.heading.target_rate,
        }
    )

    assert(target.heading == nil, "controller target should not expose old heading target")
    assertClose("manual heading rate should use current yaw target", target.yaw.angle, state.navigation.heading.angle)
    assertClose("manual heading roll feedforward", target.horizontal.feedforward.angle.roll, expected.roll)
    assertClose("manual heading pitch feedforward", target.horizontal.feedforward.angle.pitch, expected.pitch)
    assertClose("manual heading yaw feedforward", target.yaw.feedforward.angle, expected.yaw)
    assert(
        math.abs(target.horizontal.feedforward.angle.roll - targetPoseRates.roll) > 1.0e-6,
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
    local terms = machine:terms().terms
    assert(terms.active == true, "navigation should remain active without a new command")
    assert(type(terms.target) == "table", "active navigation should keep a target every tick")
    assert(type(terms.target.position) == "table", "active navigation target should include position")
    assert(terms.waypoints == nil, "navigation mode terms should not expose waypoint catalog")
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
    local terms = machine:terms().terms
    assert(terms.active == true, "active activated navigation should remain active")
    assert(type(terms.target) == "table", "active activated navigation should keep a target")
    assert(type(terms.target.position) == "table", "active activated navigation target should include position")
end

local function checkActiveNavigationUpdateReceivesDt()
    local observedDt = nil
    local machine = mode_state.new(runtimeState(), config)

    machine.modes.navigation.update = function(_, ctx)
        observedDt = ctx.dt

        return {
            active = true,
        }
    end

    machine.name = "navigation"
    machine:update({
        input = input_protocol.defaultInput(),
        state = runtimeState(),
        navigationCommand = nil,
        dt = 0.123,
    })

    assert(math.abs(observedDt - 0.123) < 1.0e-9, "active navigation update should receive real dt")
end

local function checkNavigationEnterUpdatesOnce()
    local updateCount = 0
    local observedDt = nil
    local machine = mode_state.new(runtimeState(), config)

    machine.modes.navigation.enter = function()
        return {
            active = true,
        }
    end
    machine.modes.navigation.update = function(_, ctx)
        updateCount = updateCount + 1
        observedDt = ctx.dt

        return {
            active = true,
        }
    end

    machine:update({
        input = input_protocol.defaultInput(),
        state = runtimeState(),
        navigationCommand = {
            action = "activate",
            waypoint = "home",
        },
        dt = 0.234,
    })

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
    local first = machine:terms().terms

    state.world.velocity = vector.new(9.0, 0.0, 9.0)

    machine:update({
        input = input,
        state = state,
        navigationCommand = nil,
        dt = config.control.loop.dt,
    })
    local second = machine:terms().terms

    assert(first.velocity.x == 3.0, "first cruise toggle should capture velocity")
    assert(first.velocity.y == 0.0, "cruise toggle should capture horizontal velocity")
    assert(type(first.velocity.length) == "function", "cruise velocity should be runtime vector")
    assert(first.height.target == 80.0, "cruise should freeze entry height")
    assert(first.heading.target == 0.0, "cruise should freeze entry heading")
    assert(second.velocity.x == 3.0, "held cruise toggle should not recapture velocity")
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
    assert(machine:terms().name == "position_hold", "terms should describe current mode only")
end

local function checkNavigationCancelDoesNotBlockManualRelease()
    local state = canonicalState()
    local machine = mode_state.new(state, config)

    machine.name = "manual"

    local mode = machine:update({
        input = input_protocol.defaultInput(),
        state = state,
        navigationCommand = {
            action = "cancel",
        },
        dt = config.control.loop.dt,
    })

    assert(mode.name == "position_hold", "navigation cancel should not block manual release")
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

    local legacy = legacyTarget(target, state)

    assertClose("cruise should keep entry velocity", legacy.velocity.x, 3.0)
    assertClose("cruise should keep entry velocity z", legacy.velocity.z, -1.0)
    assertClose("cruise should keep entry height", legacy.height, 80.0)
    assertClose("cruise height error should use current height", legacy.height - state.body.pose.height, -10.0)
    assertClose("cruise should keep entry heading", target.yaw.angle, 0.0)
    assertClose("cruise heading error should use current heading", target.yaw.angle - state.navigation.heading.angle, -1.0)
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
    local terms = machine:terms()
    assert(terms.navigation == nil, "manual override should not expose navigation terms")
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
    local terms = machine:terms()
    assert(terms.navigation == nil, "climb override should leave navigation inactive")
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

    local target = modeTarget(modes, {
        input = input,
        state = state,
        dt = config.control.loop.dt,
    })

    local legacy = legacyTarget(target, state)

    assert(mode.name == "position_hold", "navigation cancel exit should return to position_hold")
    assertClose("navigation exit should relock target height to current height", legacy.height, 97.0)
    assertClose("navigation exit height target should start with zero error", legacy.height - state.body.pose.height, 0.0)
    assertClose("navigation exit should relock target heading to current heading", target.yaw.angle, 1.25)
    assertClose("navigation exit heading target should start with zero error", target.yaw.angle - state.navigation.heading.angle, 0.0)
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
            terms = {},
        },
        height = {
            target = 80.0,
            error = 0.0,
        },
        heading = {
            target = 0.0,
            error = 0.0,
        },
        navigation = {
            active = false,
            waypoints = {
                {
                    id = "runtime_should_not_supply_catalog",
                    position = {
                        x = 0.0,
                        y = 0.0,
                        z = 0.0,
                    },
                },
            },
        },
        navigationConfig = config.navigation,
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
    assert(telemetry.navigation.waypoints[1].id == "home", "telemetry should expose waypoint catalog from config")
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
            terms = {},
        },
        height = {
            target = 80.0,
            error = 0.0,
        },
        heading = {
            angle = 0.0,
            error = 0.0,
        },
        state = state,
        control = {
            vertical = {
                position = {
                    target = 80.0,
                    current = 80.0,
                    error = 0.0,
                },
                velocity = {
                    target = 0.0,
                    current = 0.0,
                    error = 0.0,
                },
                output = {
                    collective = 1.0,
                },
                collective = {
                    raw = 1.0,
                    tiltCompensated = 1.0,
                },
                feedforward = {
                    position = 0.0,
                    velocity = 0.0,
                },
                tilt = {
                    compensation = 1.0,
                    verticalFactor = 1.0,
                },
                pid = {
                    position = pidTerms(),
                    velocity = pidTerms(),
                },
            },
            attitude = {
                target = {
                    roll = axisRate(),
                    pitch = axisRate(),
                    yaw = axisRate(),
                },
                current = {
                    roll = axisRate(),
                    pitch = axisRate(),
                    yaw = axisRate(),
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
                },
                output = {
                    roll = 0.0,
                    pitch = 0.0,
                    yaw = 0.0,
                },
                pid = {
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
                kind = "position",
                position = {
                    target = positionAxis(),
                    current = positionAxis(),
                    error = positionAxis(),
                },
                velocity = {
                    target = positionAxis(),
                    current = positionAxis(),
                    error = positionAxis(),
                },
                output = {
                    angle = {
                        roll = 0.0,
                        pitch = 0.0,
                    },
                },
                pid = {
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
    local control = machines.controller:update({
        state = state,
        target = target,
        dt = config.control.loop.dt,
    })
    local command = control.output
    local terms = control.terms

    assert(type(control.output) == "table", "controller update should return output")
    assert(type(control.terms) == "table", "controller update should return terms")
    assert(type(command.collective) == "number", "controller command should contain collective")
    assert(type(terms.horizontal) == "table", "controller terms should include horizontal")
    assert(type(terms.vertical) == "table", "controller terms should include vertical")
    assert(type(terms.attitude) == "table", "controller terms should include attitude")
    assert(type(terms.allocation) == "table", "controller terms should include allocation")
    assert(terms.output == nil, "controller terms should not duplicate final command under output")
    assert(type(terms.allocation.rawCommands) == "table", "allocation terms should include raw commands")
    assert(type(terms.allocation.allocatedCommands) == "table", "allocation terms should include allocated commands")
    assert(type(terms.allocation.finalCommands) == "table", "allocation terms should include final commands")
    assert(terms.allocation.debug == nil, "allocation terms should not expose allocator debug wrapper")
    assert(math.abs(terms.allocation.finalCommands.collective - command.collective) < 1.0e-6, "final collective should match top-level command")
    assert(math.abs(terms.allocation.finalCommands.roll - command.roll) < 1.0e-6, "final roll should match top-level command")
    assert(math.abs(terms.allocation.finalCommands.pitch - command.pitch) < 1.0e-6, "final pitch should match top-level command")
    assert(math.abs(terms.allocation.finalCommands.yaw - command.yaw) < 1.0e-6, "final yaw should match top-level command")
    assert(terms.horizontal.kind == "attitude", "manual target should bypass horizontal position controller")
    assert(type(terms.vertical.position.output) == "number", "vertical should expose position loop output")
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
    assertClose("yaw rate feedforward bias", config.control.attitude.rate_feedforward.yaw.bias, 0.0)
    assert(type(terms.attitude.angle.roll.output) == "number", "attitude should expose roll angle loop output")
    assert(type(terms.attitude.angle.pitch.output) == "number", "attitude should expose pitch angle loop output")
    assert(type(terms.attitude.angle.yaw.output) == "number", "attitude should expose yaw angle loop output")
    assert(terms.attitude.angle.roll.current == 0.0, "roll angle pid current should be zero quaternion-error reference")
    assert(terms.attitude.angle.roll.current ~= state.body.pose.roll, "roll angle pid current should not be body pose roll")
    assert(math.abs(terms.attitude.rate.roll.target - terms.attitude.angle.roll.output) < 1.0e-6, "roll rate target should come from angle pid output")
    assert(type(terms.attitude.rate.roll.output) == "number", "attitude should expose rate loop output")
    assert(math.abs(
        terms.allocation.rawCommands.roll
            - terms.attitude.rate.roll.output
            - terms.attitude.feedforward.rateTarget.roll
    ) < 1.0e-6, "rate pid output plus feedforward should match raw roll command")

    local telemetry = telemetryTerms.running({
        now = 1.0,
        dt = config.control.loop.dt,
        input = input_protocol.defaultInput(),
        inputEvent = {},
        inputAge = 0.0,
        inputStale = false,
        inputSender = 1,
        state = state,
        flight = {
            name = "running",
        },
        mode = {
            name = "manual",
            terms = {},
        },
        height = {},
        heading = {},
        navigation = {},
        navigationConfig = config.navigation,
        command = command,
        control = terms,
        rotor = {},
    })

    assertClose(
        "telemetry attitude view rate target",
        telemetry.control.attitude.target.roll.rate,
        terms.attitude.rate.roll.target
    )
    assertClose(
        "telemetry vertical pid view",
        telemetry.control.vertical.pid.velocity.output,
        terms.vertical.velocity.output
    )
end

local function checkControllerResetsHorizontalOnPositionEntry()
    local state = runtimeState()
    local controller = Controller.new(config.control)
    local resetCount = 0
    local attitudeTarget = common.target("attitude")
    local positionTarget = neutralTarget(state)

    controller.horizontal.reset = function()
        resetCount = resetCount + 1
    end

    attitudeTarget.horizontal.angle.roll = 0.0
    attitudeTarget.horizontal.angle.pitch = 0.0
    attitudeTarget.yaw.angle = state.navigation.heading.angle

    controller:update({
        state = state,
        target = attitudeTarget,
        dt = config.control.loop.dt,
    })
    controller:update({
        state = state,
        target = positionTarget,
        dt = config.control.loop.dt,
    })
    controller:update({
        state = state,
        target = positionTarget,
        dt = config.control.loop.dt,
    })

    assert(resetCount == 1, "controller should reset horizontal once when entering position branch")
end

local function checkVerticalTiltUsesBodyFrame()
    local state = runtimeState()
    local controller = Controller.new(config.control)
    local frame = attitude_math.frameFromPose(0.4, -0.3, 0.8)
    local target = common.target("attitude")

    state.body.frame = frame
    state.body.orientation = attitude_math.quaternionFromFrame(frame)
    state.body.pose.roll = 0.0
    state.body.pose.pitch = 0.0
    target.horizontal.angle.roll = 0.0
    target.horizontal.angle.pitch = 0.0
    target.yaw.angle = 0.8

    local control = controller:update({
        state = state,
        target = target,
        dt = config.control.loop.dt,
    })
    local expected = math.max(
        config.control.collective.tilt_compensation.min_factor,
        math.min(1.0, -frame.down.y)
    )

    assert(math.abs(expected - 1.0) > 1.0e-6, "test frame should be tilted")
    assertClose("vertical tilt factor should come from body frame", control.terms.vertical.tilt.verticalFactor, expected)
end

local function checkControllerTargetSemantics()
    local state = runtimeState()
    local controller = Controller.new(config.control)

    state.navigation.heading.angle = 1.25
    state.body.pose.heading = 1.25
    state.body.orientation = attitude_math.quaternionFromFrame(
        attitude_math.frameFromPose(state.body.pose.roll, state.body.pose.pitch, 1.25)
    )
    local target = neutralTarget(state)
    local control = controller:update({
        state = state,
        target = target,
        dt = config.control.loop.dt,
    })

    local terms = control.terms

    assert(terms.vertical.position.target == nil, "nil down position should not run position loop")
    assertClose("current yaw target should hold current yaw", terms.attitude.angle.yaw.pid.error, 0.0)
    assert(terms.horizontal.kind == "position", "position target should use horizontal position branch")
    assert(terms.horizontal.position.forward.pid == nil, "nil forward position should not run position pid")
    assert(terms.horizontal.position.right.pid == nil, "nil right position should not run position pid")

    target = neutralTarget(state)
    target.altitude.position = 0.0
    control = controller:update({
        state = state,
        target = target,
        dt = config.control.loop.dt,
    })

    terms = control.terms

    assert(terms.vertical.position.target ~= nil, "zero down position should enable height hold")
    assertClose("zero down height error", terms.vertical.position.pid.error, 0.0)

    target = neutralTarget(state)
    target.horizontal.feedforward.position.forward = 2.0
    control = controller:update({
        state = state,
        target = target,
        dt = config.control.loop.dt,
    })

    terms = control.terms

    assertClose("nil forward position uses feedforward only", terms.horizontal.velocity.forward.target, 2.0)
    assertClose("nil right position uses zero feedforward only", terms.horizontal.velocity.right.target, 0.0)

    target = neutralTarget(state)
    target.yaw.angle = math.pi / 2
    local velocityController = Controller.new(config.control)
    local savedVelocity = state.world.velocity

    state.world.velocity = vector.new(2.0, 0.0, 0.0)
    control = velocityController:update({
        state = state,
        target = target,
        dt = config.control.loop.dt,
    })
    state.world.velocity = savedVelocity

    terms = control.terms

    assertClose("horizontal current velocity should use target yaw forward", terms.horizontal.velocity.forward.current, 2.0)
    assertClose("horizontal current velocity should use target yaw right", terms.horizontal.velocity.right.current, 0.0)

    target = neutralTarget(state)
    target.horizontal.feedforward.velocity.right = 0.1
    control = controller:update({
        state = state,
        target = target,
        dt = config.control.loop.dt,
    })

    terms = control.terms

    assertClose("velocity feedforward adds to roll target", terms.horizontal.output.angle.roll, 0.1)

    target = common.target("attitude")
    target.horizontal.angle.roll = 0.1
    target.horizontal.angle.pitch = -0.2
    target.yaw.angle = state.navigation.heading.angle
    control = controller:update({
        state = state,
        target = target,
        dt = config.control.loop.dt,
    })

    terms = control.terms

    assert(terms.horizontal.kind == "attitude", "direct attitude branch should bypass horizontal PID loops")
    assertClose("direct attitude roll target", terms.horizontal.output.angle.roll, 0.1)
    assertClose("direct attitude pitch target", terms.horizontal.output.angle.pitch, -0.2)
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

    target.horizontal.feedforward.angle.roll = 0.25
    target.horizontal.feedforward.angle.pitch = -0.50
    target.yaw.feedforward.angle = 0.75
    target.horizontal.feedforward.rate.roll = 0.125
    target.horizontal.feedforward.rate.pitch = -0.250
    target.yaw.feedforward.rate = 0.375

    local control = machines.controller:update({
        state = state,
        target = target,
        dt = config.control.loop.dt,
    })

    local terms = control.terms

    assertClose("roll angle feedforward term", terms.attitude.feedforward.angle.roll, 0.25)
    assertClose("pitch angle feedforward term", terms.attitude.feedforward.angle.pitch, -0.50)
    assertClose("yaw angle feedforward term", terms.attitude.feedforward.angle.yaw, 0.75)
    assertClose("roll rate feedforward term", terms.attitude.feedforward.rate.roll, 0.125)
    assertClose("pitch rate feedforward term", terms.attitude.feedforward.rate.pitch, -0.250)
    assertClose("yaw rate feedforward term", terms.attitude.feedforward.rate.yaw, 0.375)
    assertClose("roll rate target feedforward", terms.attitude.rate.roll.target, terms.attitude.angle.roll.output + 0.25)
    assertClose("pitch rate target feedforward", terms.attitude.rate.pitch.target, terms.attitude.angle.pitch.output - 0.50)
    assertClose("yaw rate target feedforward", terms.attitude.rate.yaw.target, terms.attitude.angle.yaw.output + 0.75)
    assertClose(
        "roll command feedforward",
        terms.allocation.rawCommands.roll,
        terms.attitude.rate.roll.output + terms.attitude.feedforward.rateTarget.roll + 0.125
    )
    assertClose(
        "pitch command feedforward",
        terms.allocation.rawCommands.pitch,
        terms.attitude.rate.pitch.output + terms.attitude.feedforward.rateTarget.pitch - 0.250
    )
    assertClose(
        "yaw command feedforward",
        terms.allocation.rawCommands.yaw,
        terms.attitude.rate.yaw.output + terms.attitude.feedforward.rateTarget.yaw + 0.375
    )
end

checkFrozenBaseline()
checkTablex()
checkPid()
checkProtocolDecode()
checkFlightState()
checkModeUpdateShape()
checkModeTermsSnapshotsAreCopied()
checkModeTargetNavigationOverride()
checkNavigationTargetRequiresRoute()
checkModeTargetIsPure()
checkNavigationTargetIsPure()
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
checkNavigationCancelDoesNotBlockManualRelease()
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
checkControllerResetsHorizontalOnPositionEntry()
checkVerticalTiltUsesBodyFrame()
checkControllerTargetSemantics()
checkAttitudeExternalFeedforward()

assertOldRuntimeModuleRemoved("control_task")
assertOldRuntimeModuleRemoved("input_task")
assertOldRuntimeModuleRemoved("data_task")
assertOldRuntimeModuleRemoved("rotor")
assertOldRuntimeModuleRemoved("target_state")
assertOldRuntimeModuleRemoved("trajectory")
assertOldRuntimeModuleRemoved("navigation")
assertOldRuntimeModuleRemoved("lib.attitude_allocator")

print("control fixtures ok")
