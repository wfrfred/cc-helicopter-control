-- flight_controller/test.lua
--
-- Existing-controller inner-rate PID benchmark.
--
-- Design boundary:
--   - Reuses data_task for state estimation.
--   - Reuses controller:update() for vertical loop, attitude rate PID, output telemetry.
--   - Reuses attitude_allocator + rotor mixer/output path.
--   - Does NOT reimplement controller PID math.
--
-- Inner-loop benchmark trick:
--   For the tested axis only, replace that axis angle PID with a small fake object
--   whose update() output is the desired target body rate. This means:
--       controller:update()
--           -> existing updateAngleRate()
--           -> existing axis.rate PID
--           -> existing controller output
--           -> existing allocator/mixer
--   Non-tested axes still use the normal angle+rate loops for self-level.
--
-- Run from flight_controller:
--   test
-- or:
--   lua test.lua

local config = require("config")
local data_task = require("data_task")
local Controller = require("controller")
local attitude_math = require("lib.attitude_math")
local attitude_allocator = require("lib.attitude_allocator")
local mathx = require("lib.mathx")
local rotor = require("rotor")

assert(sublevel, "CC:Sable sublevel API not found")

-- =========================
-- Test configuration
-- =========================

local TEST = {
    log_path = "rate_benchmark.csv",

    axes = { "roll", "pitch", "yaw" },

    -- Conservative first-run amplitudes. Increase after confirming safe behavior.
    amplitude = {
        roll = math.rad(20),
        pitch = math.rad(20),
        yaw = math.rad(20),
    },

    warmup_sec = 6.0,
    recover_sec = 2.5,

    pulse = {
        duration = 3.0,
        start = 0.7,
        width = 0.45,
    },

    step = {
        duration = 4.0,
        start = 0.7,
    },

    sine = {
        duration = 8.0,
        start = 0.7,
        frequency = 0.25,
    },

    restart_startup = true,
}

local control = config.control
local loopDt = control.loop.dt

-- =========================
-- Generic helpers
-- =========================

local function clampDt(dt)
    if dt <= 0 then
        return loopDt
    end
    return math.min(dt, control.loop.max_dt)
end

local function deg(x)
    return math.deg(x or 0.0)
end

local function fmt(x)
    if x == nil then
        return ""
    end
    return string.format("%.6f", x)
end

local function csvLine(fields)
    local out = {}
    for i, v in ipairs(fields) do
        out[i] = tostring(v)
    end
    return table.concat(out, ",")
end

local function clampCommandAxis(x, lo, hi)
    if x < lo then return lo end
    if x > hi then return hi end
    return x
end

local function copyCommands(commands)
    return {
        collective = commands.collective,
        roll = commands.roll,
        pitch = commands.pitch,
        yaw = commands.yaw,
    }
end

local function finalClampCommands(commands, limits)
    return {
        collective = commands.collective,
        roll = clampCommandAxis(commands.roll, limits.roll_min, limits.roll_max),
        pitch = clampCommandAxis(commands.pitch, limits.pitch_min, limits.pitch_max),
        yaw = clampCommandAxis(commands.yaw, limits.yaw_min, limits.yaw_max),
    }
end

local function stateReady(state)
    return state ~= nil
        and state.raw ~= nil
        and state.body ~= nil
        and state.raw.position ~= nil
        and state.raw.velocity ~= nil
        and state.body.frame ~= nil
        and state.body.orientation ~= nil
        and state.body.pose ~= nil
        and state.body.rates ~= nil
        and state.body.velocity ~= nil
        and state.time ~= nil
        and state.time.pose ~= nil
        and state.time.rates ~= nil
        and state.time.velocity ~= nil
end

local function waitForState(shared)
    while shared.running and not stateReady(shared.state) do
        sleep(0.05)
    end
    assert(stateReady(shared.state), "sensor state not ready")
end

local function makeControlState(state)
    local pose = state.body.pose
    return {
        bodyFrame = state.body.frame,
        orientation = state.body.orientation,
        pose = pose,
        rates = state.body.rates,
        vertical = {
            height = pose.height,
            speed = state.raw.velocity.y,
        },
    }
end

local function targetOrientation(roll, pitch, heading)
    return attitude_math.quaternionFromFrame(
        attitude_math.frameFromPose(roll, pitch, heading)
    )
end

local function makeTarget(roll, pitch, heading, height)
    return {
        attitude = {
            roll = roll,
            pitch = pitch,
            source = "benchmark_self_level",
            orientation = targetOrientation(roll, pitch, heading),
        },
        vertical = {
            height = height,
            speed = 0.0,
            active = true,
            pending = false,
            error = nil,
            source = "benchmark_height_hold",
        },
    }
end

local function allocateCommands(result, pose)
    local rawCommands = copyCommands(result.commands)
    local allocated = attitude_allocator.apply(control.attitude_allocator, pose, rawCommands)
    local finalCommands = finalClampCommands(allocated.commands, control.output_limits)

    result.commands = finalCommands
    result.output.commands = finalCommands
    result.output.rawCommands = rawCommands
    result.output.allocatedCommands = allocated.commands
    result.output.finalCommands = finalCommands
    result.output.attitudeAllocator = allocated.debug

    local attitude = result.output.attitude
    attitude.roll.controllerCommand = rawCommands.roll
    attitude.pitch.controllerCommand = rawCommands.pitch
    attitude.yaw.controllerCommand = rawCommands.yaw

    attitude.roll.allocatedCommand = allocated.commands.roll
    attitude.pitch.allocatedCommand = allocated.commands.pitch
    attitude.yaw.allocatedCommand = allocated.commands.yaw

    attitude.roll.command = finalCommands.roll
    attitude.pitch.command = finalCommands.pitch
    attitude.yaw.command = finalCommands.yaw

    return rawCommands, allocated.commands, finalCommands, allocated.debug
end

-- =========================
-- Fake angle PID for inner-loop benchmark
-- =========================

local FakeAnglePid = {}
FakeAnglePid.__index = FakeAnglePid

function FakeAnglePid.new()
    return setmetatable({
        output = 0.0,
        lastResult = nil,
    }, FakeAnglePid)
end

function FakeAnglePid:setOutput(output)
    self.output = output or 0.0
end

function FakeAnglePid:update(input)
    local result = {
        target = input.target,
        current = input.current,
        error = input.error,
        derivative = input.derivative,
        output = self.output,
        terms = {
            p = 0.0,
            i = 0.0,
            d = 0.0,
            raw = 0.0,
            ff = 0.0,
            output = self.output,
        },
    }

    self.lastResult = result
    return result
end

function FakeAnglePid:reset()
    self.output = 0.0
    self.lastResult = nil
end

function FakeAnglePid:terms()
    return {
        p = 0.0,
        i = 0.0,
        d = 0.0,
        raw = 0.0,
        ff = 0.0,
        output = self.output,
        testOverride = true,
    }
end

local function installFakeAnglePid(controllerPids, axis, fake)
    local original = controllerPids.attitude[axis].angle
    controllerPids.attitude[axis].angle = fake
    return original
end

local function restoreAnglePid(controllerPids, axis, original)
    if original ~= nil then
        controllerPids.attitude[axis].angle = original
    end
end

local function resetAttitudePids(controllerPids)
    for _, axis in ipairs({ "roll", "pitch", "yaw" }) do
        controllerPids.attitude[axis].angle:reset()
        controllerPids.attitude[axis].rate:reset()
    end
end

-- =========================
-- CSV logging
-- =========================

local function writeHeader(log)
    log.writeLine(csvLine({
        "t",
        "phase",
        "case",
        "axis",

        "target_rate_rad",
        "target_rate_deg",
        "current_rate_rad",
        "current_rate_deg",
        "rate_error_rad",
        "rate_error_deg",

        "rate_output",
        "rate_feedback_raw",
        "rate_feedforward",

        "angle_error_rad",
        "angle_error_deg",
        "angle_output_rate_rad",
        "angle_output_rate_deg",

        "pose_roll_deg",
        "pose_pitch_deg",
        "pose_heading_deg",

        "raw_cmd_collective",
        "raw_cmd_roll",
        "raw_cmd_pitch",
        "raw_cmd_yaw",

        "allocated_cmd_collective",
        "allocated_cmd_roll",
        "allocated_cmd_pitch",
        "allocated_cmd_yaw",

        "final_cmd_collective",
        "final_cmd_roll",
        "final_cmd_pitch",
        "final_cmd_yaw",

        "height",
        "vertical_speed",
    }))
end

local function writeSample(log, sample)
    local axis = sample.axis
    local outputAxis = sample.result.output.attitude[axis]
    local targetAxis = sample.result.target.attitude[axis]
    local currentAxis = sample.result.current.attitude[axis]
    local errorAxis = sample.result.error.attitude[axis]

    log.writeLine(csvLine({
        fmt(sample.t),
        sample.phase,
        sample.caseName or "",
        axis,

        fmt(targetAxis.rate),
        fmt(deg(targetAxis.rate)),
        fmt(currentAxis.rate),
        fmt(deg(currentAxis.rate)),
        fmt(errorAxis.rate),
        fmt(deg(errorAxis.rate)),

        fmt(outputAxis.command),
        fmt(outputAxis.feedback),
        fmt(outputAxis.feedforward),

        fmt(errorAxis.angle),
        fmt(deg(errorAxis.angle)),
        fmt(outputAxis.angleRate),
        fmt(deg(outputAxis.angleRate)),

        fmt(deg(sample.pose.roll)),
        fmt(deg(sample.pose.pitch)),
        fmt(deg(sample.pose.heading)),

        fmt(sample.rawCommands.collective),
        fmt(sample.rawCommands.roll),
        fmt(sample.rawCommands.pitch),
        fmt(sample.rawCommands.yaw),

        fmt(sample.allocatedCommands.collective),
        fmt(sample.allocatedCommands.roll),
        fmt(sample.allocatedCommands.pitch),
        fmt(sample.allocatedCommands.yaw),

        fmt(sample.finalCommands.collective),
        fmt(sample.finalCommands.roll),
        fmt(sample.finalCommands.pitch),
        fmt(sample.finalCommands.yaw),

        fmt(sample.pose.height),
        fmt(sample.verticalSpeed),
    }))
end

-- =========================
-- Rotor safety
-- =========================

local mixer = nil
local lastCollective = control.vertical.feedforward.bias or 1.0

local function setMixerCommands(commands)
    lastCollective = commands.collective or lastCollective
    mixer:setCommands(commands)
    mixer:update()
end

local function neutralizeBriefly()
    if mixer == nil then
        return
    end

    local neutral = {
        collective = lastCollective,
        roll = 0.0,
        pitch = 0.0,
        yaw = 0.0,
    }

    for _ = 1, 3 do
        pcall(function()
            mixer:setCommands(neutral)
            mixer:update()
        end)
        sleep(0.05)
    end
end

-- =========================
-- Target generators
-- =========================

local function pulseTarget(axis)
    local amp = TEST.amplitude[axis]
    return function(t)
        if t >= TEST.pulse.start and t < TEST.pulse.start + TEST.pulse.width then
            return amp
        end
        return 0.0
    end
end

local function stepTarget(axis)
    local amp = TEST.amplitude[axis]
    return function(t)
        if t >= TEST.step.start then
            return amp
        end
        return 0.0
    end
end

local function sineTarget(axis)
    local amp = TEST.amplitude[axis]
    local omega = 2.0 * math.pi * TEST.sine.frequency

    return function(t)
        if t < TEST.sine.start then
            return 0.0
        end
        return amp * math.sin(omega * (t - TEST.sine.start))
    end
end

-- =========================
-- Benchmark loop
-- =========================

local function runControlPhase(ctx, phase, caseName, testAxis, duration, targetFunc)
    local start = os.clock()
    local last = start

    while ctx.shared.running do
        local now = os.clock()
        local elapsed = now - start
        if elapsed >= duration then
            break
        end

        local dt = clampDt(now - last)
        last = now

        local state = ctx.shared.state
        if not stateReady(state) then
            sleep(0.05)
        else
            local pose = state.body.pose
            local controlState = makeControlState(state)

            local holdHeading = ctx.holdHeading

            if testAxis == "yaw" then
                -- For yaw-rate tests, avoid building up a large yaw attitude error.
                -- Roll/pitch still self-level against current heading.
                holdHeading = pose.heading
            end

            local target = makeTarget(0.0, 0.0, holdHeading, ctx.holdHeight)

            local forcedRate = 0.0
            if targetFunc ~= nil then
                forcedRate = targetFunc(elapsed)
                ctx.fakeAngle:setOutput(forcedRate)
            else
                ctx.fakeAngle:setOutput(0.0)
            end

            local result = ctx.controller:update({
                target = target,
                state = controlState,
                dt = dt,
            })

            local rawCommands, allocatedCommands, finalCommands =
                allocateCommands(result, pose)

            setMixerCommands(result.commands)

            ctx.shared.target = target
            ctx.shared.controlResult = result
            ctx.shared.commands = result.commands

            if testAxis ~= nil then
                writeSample(ctx.log, {
                    t = elapsed,
                    phase = phase,
                    caseName = caseName,
                    axis = testAxis,
                    result = result,
                    pose = pose,
                    verticalSpeed = state.raw.velocity.y,
                    rawCommands = rawCommands,
                    allocatedCommands = allocatedCommands,
                    finalCommands = finalCommands,
                })
            end

            local used = os.clock() - now
            local wait = loopDt - used
            if wait > 0 then
                sleep(wait)
            else
                sleep(0)
            end
        end
    end
end

local function stabilize(ctx, label, duration)
    print(label .. "...")
    ctx.fakeAngle:setOutput(0.0)
    runControlPhase(ctx, label, "", nil, duration, nil)

    if stateReady(ctx.shared.state) then
        ctx.holdHeading = ctx.shared.state.body.pose.heading
    end
end

local function runOneCase(ctx, axis, case)
    local original = installFakeAnglePid(ctx.controllerPids, axis, ctx.fakeAngle)

    ctx.fakeAngle:reset()
    ctx.controllerPids.attitude[axis].rate:reset()

    print("benchmark " .. axis .. " " .. case.name)

    local ok, err = pcall(function()
        runControlPhase(
            ctx,
            "benchmark",
            case.name,
            axis,
            case.duration,
            case.target(axis)
        )
    end)

    restoreAnglePid(ctx.controllerPids, axis, original)
    ctx.fakeAngle:reset()
    ctx.controllerPids.attitude[axis].rate:reset()

    if not ok then
        error(err)
    end
end

local function runBenchmarks(ctx)
    print("warmup self-level / height hold")
    stabilize(ctx, "warmup", TEST.warmup_sec)

    local cases = {
        {
            name = "pulse",
            duration = TEST.pulse.duration,
            target = pulseTarget,
        },
        {
            name = "step",
            duration = TEST.step.duration,
            target = stepTarget,
        },
        {
            name = "sine",
            duration = TEST.sine.duration,
            target = sineTarget,
        },
    }

    for _, axis in ipairs(TEST.axes) do
        for _, case in ipairs(cases) do
            runOneCase(ctx, axis, case)
            stabilize(ctx, "recover_" .. axis .. "_" .. case.name, TEST.recover_sec)
        end
    end
end

local function benchmarkTask(shared)
    waitForState(shared)

    local initial = shared.state.body.pose

    local controller = Controller.new(config.control)
    local controllerPids = controller:pidControllers()

    local ctx = {
        shared = shared,
        controller = controller,
        controllerPids = controllerPids,
        fakeAngle = FakeAnglePid.new(),
        holdHeight = initial.height,
        holdHeading = initial.heading,
        log = assert(fs.open(TEST.log_path, "w")),
    }

    writeHeader(ctx.log)
    resetAttitudePids(ctx.controllerPids)

    runBenchmarks(ctx)

    ctx.log.close()
    print("benchmark finished: " .. TEST.log_path)

    shared.running = false
end

local function main()
    term.clear()
    term.setCursorPos(1, 1)

    print("rate benchmark test.lua")
    print("state: existing data_task")
    print("control: existing controller:update")
    print("output: existing allocator + rotor")
    print("log: " .. TEST.log_path)
    print("will restart startup when finished")

    mixer = rotor.new(config.hardware.rotor, config.calibration.rotor)

    local shared = {
        state = nil,
        running = true,
    }

    parallel.waitForAny(
        function()
            data_task.run(shared)
        end,
        function()
            benchmarkTask(shared)
        end
    )

    shared.running = false
end

local ok, err = pcall(main)

if not ok then
    print("test.lua failed:")
    print(tostring(err))
end

neutralizeBriefly()

if TEST.restart_startup then
    print("restarting startup...")
    sleep(0.2)
    shell.run("startup")
end

if not ok then
    error(err)
end
