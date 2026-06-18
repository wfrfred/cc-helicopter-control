-- flight_controller/velocity_step_zero_benchmark.lua
--
-- Forward/right velocity step benchmark for cc-helicopter-control master.
--
-- Cases:
--   forward +8 m/s -> 0
--   forward -8 m/s -> 0
--   right   +8 m/s -> 0
--   right   -8 m/s -> 0
--
-- The step_out half is mainly for velocity feedforward estimation.
-- The return_zero half is mainly for velocity PID braking / recovery response.

local config = require("config")
local data_task = require("data_task")
local Controller = require("controller")
local position_hold = require("position_hold")
local attitude_math = require("lib.attitude_math")
local attitude_allocator = require("lib.attitude_allocator")
local mathx = require("lib.mathx")
local rotor = require("rotor")

assert(sublevel, "CC:Sable sublevel API not found")

local TEST = {
    log_path = "velocity_step_zero_benchmark.csv",

    axes = { "forward", "right" },
    signs = { 1.0, -1.0 },

    amplitude = 5.0,

    warmup_sec = 5.0,
    hold_sec = 5.0,
    return_sec = 5.0,
    recover_sec = 4.0,

    restart_startup = true,
}

local control = config.control
local loopDt = control.loop.dt

local function clampDt(dt)
    if dt <= 0 then
        return loopDt
    end

    return math.min(dt, control.loop.max_dt)
end

local function fmt(x)
    if x == nil then
        return ""
    end

    if type(x) ~= "number" then
        return tostring(x)
    end

    return string.format("%.6f", x)
end

local function deg(x)
    return math.deg(x or 0.0)
end

local function getIn(value, ...)
    local current = value
    local keys = { ... }

    for _, key in ipairs(keys) do
        if type(current) ~= "table" then
            return nil
        end

        current = current[key]
    end

    return current
end

local function getNumber(value, default, ...)
    local found = getIn(value, ...)

    if type(found) == "number" then
        return found
    end

    return default or 0.0
end

local function csvLine(fields)
    local out = {}

    for i, value in ipairs(fields) do
        out[i] = tostring(value)
    end

    return table.concat(out, ",")
end

local function clampCommandAxis(x, lo, hi)
    if x < lo then
        return lo
    end

    if x > hi then
        return hi
    end

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

local function resetPidTree(value)
    if type(value) ~= "table" then
        return
    end

    if type(value.reset) == "function" then
        value:reset()
        return
    end

    for _, child in pairs(value) do
        resetPidTree(child)
    end
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
end

local function waitForState(shared)
    while shared.running and not stateReady(shared.state) do
        sleep(0.05)
    end

    assert(stateReady(shared.state), "sensor state not ready")
end

local function navigationHorizontal(forward, right)
    return {
        forward = forward,
        right = right,
    }
end

local function navigationHorizontalAxes(heading)
    return {
        right = {
            x = math.cos(heading),
            z = math.sin(heading),
        },
        forward = {
            x = math.sin(heading),
            z = -math.cos(heading),
        },
    }
end

local function projectWorldHorizontalToNavigation(value, heading)
    return mathx.project(value, navigationHorizontalAxes(heading))
end

local function projectNavigationHorizontalToWorld(value, heading)
    local axes = navigationHorizontalAxes(heading)

    return {
        x = (value.right or 0.0) * axes.right.x
            + (value.forward or 0.0) * axes.forward.x,
        z = (value.right or 0.0) * axes.right.z
            + (value.forward or 0.0) * axes.forward.z,
    }
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
    ):normalize()
end

local function makeTarget(attitude, heading, height)
    local orientation = targetOrientation(attitude.roll, attitude.pitch, heading)

    return {
        attitude = {
            roll = attitude.roll,
            pitch = attitude.pitch,
            source = "velocity_step_zero_benchmark",
            orientation = orientation,
            fullOrientation = orientation,
            reducedOrientation = orientation,
            yawPriority = 1.0,
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

    return rawCommands, allocated.commands, finalCommands
end

local function controllerTargetRate(result, axis)
    return getNumber(
        result,
        getNumber(result, 0.0, "output", "attitude", axis, "targetRate"),
        "target", "attitude", axis, "rate"
    )
end

local function controllerCurrentRate(result, axis)
    return getNumber(result, 0.0, "current", "attitude", axis, "rate")
end

local function writeHeader(log)
    log.writeLine(csvLine({
        "t",
        "segment",
        "case",
        "axis",
        "direction",

        "target_velocity",
        "current_velocity",
        "velocity_error",

        "target_forward_velocity",
        "target_right_velocity",
        "current_forward_velocity",
        "current_right_velocity",
        "forward_velocity_error",
        "right_velocity_error",

        "tilt_output",
        "tilt_feedback_raw",
        "tilt_feedforward",

        "forward_tilt_output",
        "forward_tilt_feedback_raw",
        "forward_tilt_feedforward",
        "right_tilt_output",
        "right_tilt_feedback_raw",
        "right_tilt_feedforward",

        "target_roll_deg",
        "target_pitch_deg",
        "current_roll_deg",
        "current_pitch_deg",
        "current_heading_deg",
        "target_heading_deg",
        "heading_error_deg",

        "target_roll_rate_deg",
        "target_pitch_rate_deg",
        "target_yaw_rate_deg",
        "current_roll_rate_deg",
        "current_pitch_rate_deg",
        "current_yaw_rate_deg",

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

        "world_velocity_x",
        "world_velocity_z",
        "target_world_velocity_x",
        "target_world_velocity_z",

        "height",
        "vertical_speed",
    }))
end

local function axisTilt(output, axis, field)
    return getNumber(output, 0.0, "navigationTilt", axis, field)
end

local function writeSample(log, sample)
    local axis = sample.axis
    local positionResult = sample.positionResult
    local pose = sample.pose
    local state = sample.state
    local result = sample.result

    local targetNavVelocity = positionResult.navigationVelocity.target
    local currentNavVelocity = positionResult.navigationVelocity.current
    local errorNavVelocity = positionResult.navigationVelocity.error

    local targetVelocity = targetNavVelocity[axis] or 0.0
    local currentVelocity = currentNavVelocity[axis] or 0.0
    local velocityError = targetVelocity - currentVelocity

    local output = positionResult.output
    local tiltOutput = axisTilt(output, axis, "value")
    local tiltFeedback = axisTilt(output, axis, "feedback")
    local tiltFeedforward = axisTilt(output, axis, "feedforward")

    local headingError = mathx.wrapPi(sample.holdHeading - pose.heading)

    log.writeLine(csvLine({
        fmt(sample.t),
        sample.segment,
        sample.caseName,
        axis,
        fmt(sample.direction),

        fmt(targetVelocity),
        fmt(currentVelocity),
        fmt(velocityError),

        fmt(targetNavVelocity.forward),
        fmt(targetNavVelocity.right),
        fmt(currentNavVelocity.forward),
        fmt(currentNavVelocity.right),
        fmt(errorNavVelocity.forward),
        fmt(errorNavVelocity.right),

        fmt(tiltOutput),
        fmt(tiltFeedback),
        fmt(tiltFeedforward),

        fmt(axisTilt(output, "forward", "value")),
        fmt(axisTilt(output, "forward", "feedback")),
        fmt(axisTilt(output, "forward", "feedforward")),
        fmt(axisTilt(output, "right", "value")),
        fmt(axisTilt(output, "right", "feedback")),
        fmt(axisTilt(output, "right", "feedforward")),

        fmt(deg(positionResult.output.attitude.roll)),
        fmt(deg(positionResult.output.attitude.pitch)),
        fmt(deg(pose.roll)),
        fmt(deg(pose.pitch)),
        fmt(deg(pose.heading)),
        fmt(deg(sample.holdHeading)),
        fmt(deg(headingError)),

        fmt(deg(controllerTargetRate(result, "roll"))),
        fmt(deg(controllerTargetRate(result, "pitch"))),
        fmt(deg(controllerTargetRate(result, "yaw"))),
        fmt(deg(controllerCurrentRate(result, "roll"))),
        fmt(deg(controllerCurrentRate(result, "pitch"))),
        fmt(deg(controllerCurrentRate(result, "yaw"))),

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

        fmt(state.raw.velocity.x),
        fmt(state.raw.velocity.z),
        fmt(sample.targetWorldVelocity.x),
        fmt(sample.targetWorldVelocity.z),

        fmt(pose.height),
        fmt(state.raw.velocity.y),
    }))
end

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

local function targetNavigationVelocity(axis, value)
    if axis == "forward" then
        return navigationHorizontal(value, 0.0)
    end

    if axis == "right" then
        return navigationHorizontal(0.0, value)
    end

    error("unknown velocity axis: " .. tostring(axis))
end

local function runControlPhase(ctx, segment, caseName, axis, direction, duration, velocityFunc)
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
            local navValue = velocityFunc and velocityFunc(elapsed) or 0.0
            local targetNavVelocity = targetNavigationVelocity(axis or "forward", navValue)
            local targetWorldVelocity = projectNavigationHorizontalToWorld(targetNavVelocity, ctx.holdHeading)
            local currentNavVelocity = projectWorldHorizontalToNavigation(state.raw.velocity, ctx.holdHeading)

            local positionResult = ctx.positionHold:updateVelocity(
                targetWorldVelocity,
                state.raw.velocity,
                ctx.holdHeading,
                dt,
                nil
            )

            local target = makeTarget(
                positionResult.output.attitude,
                ctx.holdHeading,
                ctx.holdHeight
            )

            local result = ctx.controller:update({
                target = target,
                state = makeControlState(state),
                dt = dt,
            })

            local rawCommands, allocatedCommands, finalCommands = allocateCommands(result, pose)

            setMixerCommands(result.commands)

            ctx.shared.positionResult = positionResult
            ctx.shared.target = target
            ctx.shared.controlResult = result
            ctx.shared.commands = result.commands

            if axis ~= nil and caseName ~= "" then
                writeSample(ctx.log, {
                    t = elapsed,
                    segment = segment,
                    caseName = caseName,
                    axis = axis,
                    direction = direction,
                    targetWorldVelocity = targetWorldVelocity,
                    currentNavVelocity = currentNavVelocity,
                    positionResult = positionResult,
                    result = result,
                    state = state,
                    pose = pose,
                    holdHeading = ctx.holdHeading,
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

    runControlPhase(ctx, label, "", "forward", 0.0, duration, function()
        return 0.0
    end)

    if stateReady(ctx.shared.state) then
        ctx.holdHeading = ctx.shared.state.body.pose.heading
    end
end

local function runOneCase(ctx, axis, direction)
    resetPidTree(ctx.positionPids)
    resetPidTree(ctx.controllerPids)

    local amplitude = TEST.amplitude * direction
    local signName = direction > 0.0 and "pos" or "neg"
    local caseName = axis .. "_" .. signName .. "_8ms_step_zero"

    print("benchmark " .. caseName)

    runControlPhase(
        ctx,
        "step_out",
        caseName,
        axis,
        direction,
        TEST.hold_sec,
        function()
            return amplitude
        end
    )

    runControlPhase(
        ctx,
        "return_zero",
        caseName,
        axis,
        direction,
        TEST.return_sec,
        function()
            return 0.0
        end
    )

    resetPidTree(ctx.positionPids)
    resetPidTree(ctx.controllerPids)
end

local function runBenchmarks(ctx)
    print("warmup zero velocity / height hold")
    stabilize(ctx, "warmup", TEST.warmup_sec)

    for _, axis in ipairs(TEST.axes) do
        for _, direction in ipairs(TEST.signs) do
            runOneCase(ctx, axis, direction)
            stabilize(ctx, "recover_" .. axis .. "_" .. tostring(direction), TEST.recover_sec)
        end
    end
end

local function benchmarkTask(shared)
    waitForState(shared)

    local initial = shared.state.body.pose
    local controller = Controller.new(config.control)
    local positionHold = position_hold.new(config.control)

    local ctx = {
        shared = shared,
        controller = controller,
        controllerPids = controller:pidControllers(),
        positionHold = positionHold,
        positionPids = positionHold:pidControllers(),
        holdHeight = initial.height,
        holdHeading = initial.heading,
        log = assert(fs.open(TEST.log_path, "w")),
    }

    writeHeader(ctx.log)
    resetPidTree(ctx.positionPids)
    resetPidTree(ctx.controllerPids)

    runBenchmarks(ctx)

    ctx.log.close()
    print("benchmark finished: " .. TEST.log_path)

    shared.running = false
end

local function main()
    term.clear()
    term.setCursorPos(1, 1)

    print("velocity step-zero benchmark")
    print("forward/right +/-" .. tostring(TEST.amplitude) .. " m/s step, then return to zero")
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

local ok, err = xpcall(main, debug.traceback)

if not ok then
    print("velocity_step_zero_benchmark.lua failed:")
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
