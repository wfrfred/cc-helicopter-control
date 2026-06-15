local Controller = require("controller")
local mathx = require("lib.mathx")
local rotor = require("rotor")
local target_state = require("target_state")
local position_hold = require("position_hold")
local rate_lock = require("rate_lock")
local config = require("config")

local control_task = {}

local function clampDt(dt)
    if dt <= 0 then
        return config.control.loop.dt
    end

    return math.min(dt, config.control.loop.max_dt)
end

local zeroInput = {
    controls = {
        roll = 0.0,
        pitch = 0.0,
        heading = 0.0,
        climb = 0.0,
    },
    event = {
        cruiseLock = false,
    },
}

local function readInput(shared, now)
    local input = shared.input
    local inputAge = now - shared.inputTime

    if inputAge > config.control.input.stale_dt then
        return zeroInput, inputAge, true
    end

    return input, inputAge, false
end

local function stateReady(state)
    return state ~= nil and
        state.raw.position ~= nil and
        state.raw.velocity ~= nil and
        state.body.frame ~= nil and
        state.body.pose ~= nil and
        state.body.rates ~= nil and
        state.body.velocity ~= nil and
        state.time.pose ~= nil and
        state.time.rates ~= nil and
        state.time.velocity ~= nil
end

local function waitForSensors(shared)
    while shared.running and not stateReady(shared.state) do
        local state = shared.state
        local haveState = state ~= nil
        local now = os.clock()

        shared.telemetryTime = now
        shared.telemetry = {
            status = "waiting_sensors",
            time = now,
            havePose = haveState and state.body.pose ~= nil and state.body.frame ~= nil,
            haveRates = haveState and state.body.rates ~= nil,
            haveVelocity = haveState and state.body.velocity ~= nil,
        }

        sleep(0.1)
    end
end

local lateralMode = {
    manual = "manual",
    cruise = "cruise",
    positionHold = "position_hold",
    navigation = "navigation",
}

local verticalMode = {
    heightHold = "height_hold",
}

local headingMode = {
    headingHold = "heading_hold",
}

local function manualLateralInput(controls)
    return controls.roll ~= 0 or controls.pitch ~= 0
end

local positionTargetAxes = {
    x = { x = 1.0 },
    z = { z = 1.0 },
}

local function headingHorizontalAxes(heading)
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

local function makePositionTarget(state)
    return mathx.project(state.raw.position, positionTargetAxes)
end

local function projectHorizontalToNavigationFrd(value, heading)
    return mathx.project(value, headingHorizontalAxes(heading))
end

local function projectPositionTargetErrorToNavigationFrd(target, state)
    local position = state.raw.position

    return projectHorizontalToNavigationFrd({
        x = target.x - position.x,
        z = target.z - position.z,
    }, state.body.pose.heading)
end

local function projectHorizontalVelocityToNavigationFrd(state)
    return projectHorizontalToNavigationFrd(state.raw.velocity, state.body.pose.heading)
end

local function makeLateralMachine(initialState)
    return {
        mode = lateralMode.positionHold,
        positionTarget = makePositionTarget(initialState),
        cruiseVelocity = nil,
        cruiseManualReleasePending = false,
        navigationTarget = nil,
    }
end

local function enterLateralMode(machine, mode, state, positionHold)
    if machine.mode == mode then
        return
    end

    machine.mode = mode
    positionHold:reset()

    if mode == lateralMode.manual or mode == lateralMode.positionHold then
        machine.positionTarget = makePositionTarget(state)
    end
end

local function selectLateralMode(machine, controls)
    if machine.navigationTarget ~= nil then
        return lateralMode.navigation
    end

    if machine.cruiseVelocity ~= nil then
        return lateralMode.cruise
    end

    if manualLateralInput(controls) then
        return lateralMode.manual
    end

    return lateralMode.positionHold
end

local function updateCruiseTarget(machine, context)
    local manualInput = manualLateralInput(context.input.controls)

    if context.input.event.cruiseLock then
        machine.cruiseVelocity = projectHorizontalVelocityToNavigationFrd(context.state)
        machine.cruiseManualReleasePending = manualInput
        context.positionHold:reset()
        context.input.event.cruiseLock = false
        return
    end

    if machine.cruiseVelocity == nil then
        return
    end

    if machine.cruiseManualReleasePending then
        if not manualInput then
            machine.cruiseManualReleasePending = false
        end
        return
    end

    if manualInput then
        machine.cruiseVelocity = nil
    end
end

local function attitudeTarget(source, attitude)
    return {
        roll = attitude.roll,
        pitch = attitude.pitch,
        source = source,
    }
end

local function manualLateral(machine, context)
    return position_hold.inactive(), context.manualAttitude:target(machine.mode)
end

local function cruiseLateral(machine, context)
    local positionResult = context.positionHold:updateVelocity(
        machine.cruiseVelocity,
        projectHorizontalVelocityToNavigationFrd(context.state),
        context.dt
    )

    return positionResult, attitudeTarget(machine.mode, positionResult.output.attitude)
end

local function positionHoldLateral(machine, context)
    local positionResult = context.positionHold:update(
        projectPositionTargetErrorToNavigationFrd(machine.positionTarget, context.state),
        projectHorizontalVelocityToNavigationFrd(context.state),
        context.dt
    )

    return positionResult, attitudeTarget(machine.mode, positionResult.output.attitude)
end

local function navigationLateral(machine, context)
    local positionResult = context.positionHold:update(
        projectPositionTargetErrorToNavigationFrd(machine.navigationTarget.position, context.state),
        projectHorizontalVelocityToNavigationFrd(context.state),
        context.dt
    )

    return positionResult, attitudeTarget(machine.mode, positionResult.output.attitude)
end

local function lateralPositionTarget(machine)
    if machine.mode == lateralMode.positionHold then
        return machine.positionTarget
    end

    if machine.mode == lateralMode.navigation then
        return machine.navigationTarget.position
    end

    return nil
end

local lateralHandlers = {
    manual = manualLateral,
    cruise = cruiseLateral,
    position_hold = positionHoldLateral,
    navigation = navigationLateral,
}

local function updateLateral(machine, context)
    updateCruiseTarget(machine, context)

    enterLateralMode(
        machine,
        selectLateralMode(machine, context.input.controls),
        context.state,
        context.positionHold
    )

    return lateralHandlers[machine.mode](machine, context)
end

local function makeControlState(state)
    local pose = state.body.pose

    return {
        frame = state.body.frame,
        pose = pose,
        rates = state.body.rates,
        vertical = {
            height = pose.height,
            speed = state.raw.velocity.y,
        },
    }
end

local function makeTarget(attitude, position, verticalLock, headingLockResult)
    return {
        attitude = attitude,
        position = position,
        vertical = {
            height = verticalLock.target,
            speed = verticalLock.commandedRate,
            active = verticalLock.active,
            pending = verticalLock.pending,
            error = verticalLock.error,
            source = verticalLock.state,
        },
        heading = {
            angle = headingLockResult.target,
            rate = headingLockResult.commandedRate,
            active = headingLockResult.active,
            pending = headingLockResult.pending,
            error = headingLockResult.error,
            source = headingLockResult.state,
        },
    }
end

local function makeTelemetryState(state)
    return {
        raw = {
            position = state.raw.position,
            velocity = state.raw.velocity,
        },
        body = {
            frame = state.body.frame,
            pose = state.body.pose,
            velocity = state.body.velocity,
            rates = state.body.rates,
        },
    }
end

function control_task.run(shared)
    local mixer = rotor.new(config.hardware.rotor, config.calibration.rotor)

    waitForSensors(shared)

    local initialState = shared.state
    local initial = initialState.body.pose

    local manualAttitude = target_state.new(initial, config.control)
    local positionHold = position_hold.new(config.control)
    local lateralMachine = makeLateralMachine(initialState)
    local heightLock = rate_lock.new({
        initial_target = initial.height,
        target_rate = config.control.vertical.target_rate,
        rate_deadband = config.control.vertical.lock.speed_deadband,
        relock_timeout = config.control.vertical.lock.relock_timeout,
    })
    local headingLock = rate_lock.new({
        initial_target = initial.heading,
        target_rate = config.control.heading.target_rate,
        rate_deadband = config.control.heading.lock.rate_deadband,
        relock_timeout = config.control.heading.lock.relock_timeout,
        error = function(target, current)
            return mathx.wrapPi(target - current)
        end,
    })
    local controller = Controller.new(config.control)
    local controllerPids = controller:pidControllers()
    local positionPids = positionHold:pidControllers()

    local lastLoopTime = os.clock() - config.control.loop.dt
    local telemetryTimer = 0.0

    while shared.running do
        local loopStart = os.clock()
        local dt = clampDt(loopStart - lastLoopTime)
        lastLoopTime = loopStart

        local input, inputAge, inputStale = readInput(shared, loopStart)
        local inputEvent = {
            cruiseLock = input.event.cruiseLock,
        }

        manualAttitude:update(input.controls, dt)

        local now = os.clock()
        local state = shared.state
        local pose = state.body.pose
        local controlState = makeControlState(state)

        local positionResult, attitudeTarget = updateLateral(lateralMachine, {
            input = input,
            state = state,
            manualAttitude = manualAttitude,
            positionHold = positionHold,
            dt = dt,
        })

        local controls = input.controls
        local vertical = controlState.vertical
        local verticalLock = heightLock:update(controls.climb, vertical.height, vertical.speed, dt)
        local headingLockResult = headingLock:update(controls.heading, pose.heading, controlState.rates.yaw, dt)
        local target = makeTarget(
            attitudeTarget,
            lateralPositionTarget(lateralMachine),
            verticalLock,
            headingLockResult
        )

        local result = controller:update({
            target = target,
            state = controlState,
            dt = dt,
        })

        mixer:setCommands(result.commands)
        local rotorOutput = mixer:update()
        result.output.rotor = rotorOutput.blades

        shared.target = target
        shared.controlResult = result
        shared.commands = result.commands

        telemetryTimer = telemetryTimer + dt
        if telemetryTimer >= config.control.loop.telemetry_dt then
            telemetryTimer = 0.0

            shared.telemetryTime = now
            shared.telemetry = {
                status = "running",
                time = now,
                dt = dt,

                age = {
                    pose = now - state.time.pose,
                    rates = now - state.time.rates,
                    velocity = now - state.time.velocity,
                },

                input = {
                    controls = input.controls,
                    event = inputEvent,
                    age = inputAge,
                    stale = inputStale,
                    sender = shared.inputSender,
                },

                mode = {
                    lateral = lateralMachine.mode,
                    vertical = verticalMode.heightHold,
                    heading = headingMode.headingHold,
                },

                lock = {
                    height = verticalLock.state,
                    heading = headingLockResult.state,
                },

                state = makeTelemetryState(state),

                output = result.output,

                pid = {
                    vertical = {
                        height = controllerPids.vertical.height:terms(),
                        speed = controllerPids.vertical.speed:terms(),
                    },
                    position = {
                        right = positionPids.positionRight:terms(),
                        forward = positionPids.positionForward:terms(),
                    },
                    velocity = {
                        right = positionPids.velocityRight:terms(),
                        forward = positionPids.velocityForward:terms(),
                    },
                    attitude = {
                        roll = {
                            angle = controllerPids.attitude.roll.angle:terms(),
                            rate = controllerPids.attitude.roll.rate:terms(),
                        },
                        pitch = {
                            angle = controllerPids.attitude.pitch.angle:terms(),
                            rate = controllerPids.attitude.pitch.rate:terms(),
                        },
                        yaw = {
                            angle = controllerPids.attitude.yaw.angle:terms(),
                            rate = controllerPids.attitude.yaw.rate:terms(),
                        },
                    },
                },

                target = result.target,

                current = result.current,

                error = result.error,

                positionHold = positionResult,
            }
        end

        local elapsed = os.clock() - loopStart
        local remain = config.control.loop.dt - elapsed

        if remain > 0 then
            sleep(remain)
        else
            sleep(0)
        end
    end
end

return control_task
