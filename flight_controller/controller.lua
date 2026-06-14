local mathx = require("lib.mathx")
local pid = require("lib.pid")

local controller = {}

local Controller = {}
Controller.__index = Controller

local function attitudeVerticalFactor(roll, pitch, minFactor)
    local factor = math.cos(roll) * math.cos(pitch)

    return mathx.clamp(factor, minFactor, 1.0)
end

function controller.new(control)
    return setmetatable({
        collectiveMin = control.collective_min,
        collectiveMax = control.collective_max,
        verticalSpeedFeedforwardGain = control.vertical_speed_feedforward_gain,
        verticalSpeedFeedforwardBias = control.vertical_speed_feedforward_bias,
        tiltCompensationMinFactor = control.tilt_compensation_min_factor,
        yawRateFeedforwardGain = control.yaw_rate_feedforward_gain,
        pitchFeedforwardBias = control.pitch_feedforward_bias,

        height = pid.new(control.pid.height),
        verticalSpeed = pid.new(control.pid.vertical_speed),
        roll = pid.new(control.pid.roll),
        pitch = pid.new(control.pid.pitch),
        yawAngle = pid.new(control.pid.yaw_angle),
        yawRate = pid.new(control.pid.yaw_rate),
    }, Controller)
end

function Controller:update(input)
    local target = input.target
    local state = input.state
    local pose = state.pose
    local velocity = state.velocity
    local rates = state.rates
    local attitudeTarget = target.attitude
    local verticalTarget = target.vertical
    local yawTarget = target.yaw
    local downSpeed = velocity.down
    local rollRate = rates.roll
    local pitchRate = rates.pitch
    local yawRate = rates.yaw
    local dt = input.dt

    local targetDownSpeed = verticalTarget.rate
    local heightErr = verticalTarget.error

    if verticalTarget.active then
        targetDownSpeed, heightErr = self.height:update(verticalTarget.down, pose.down, dt, -downSpeed)
    else
        self.height:reset()
    end

    local downSpeedFeedback, downSpeedErr = self.verticalSpeed:update(targetDownSpeed, downSpeed, dt)
    local downSpeedFeedforward = self.verticalSpeedFeedforwardGain * targetDownSpeed
    local collectiveFeedforward = self.verticalSpeedFeedforwardBias - downSpeedFeedforward
    local collectiveFeedback = -downSpeedFeedback
    local collectiveOut = collectiveFeedforward + collectiveFeedback
    local tiltVerticalFactor = attitudeVerticalFactor(
        pose.roll,
        pose.pitch,
        self.tiltCompensationMinFactor
    )
    local tiltCompensation = 1.0 / tiltVerticalFactor
    local tiltCompensatedCollectiveOut = collectiveOut * tiltCompensation

    local rollErr = mathx.wrapPi(attitudeTarget.roll - pose.roll)
    local rollCmd = self.roll:update(rollErr, 0.0, dt, -rollRate)

    local pitchErr = mathx.wrapPi(attitudeTarget.pitch - pose.pitch)
    local pitchFeedback = self.pitch:update(pitchErr, 0.0, dt, -pitchRate)
    local pitchFeedforward = self.pitchFeedforwardBias
    local pitchCmd = pitchFeedforward + pitchFeedback

    local targetYawRate = yawTarget.rate
    local yawErr = yawTarget.error
    local yawAngleActive = yawTarget.active

    if yawAngleActive then
        targetYawRate = self.yawAngle:update(yawErr, 0.0, dt, -yawRate)
    else
        self.yawAngle:reset()
    end

    local yawRateFeedback, yawRateErr = self.yawRate:update(targetYawRate, yawRate, dt)
    local yawRateFeedforward = self.yawRateFeedforwardGain * targetYawRate
    local yawCmd = yawRateFeedforward + yawRateFeedback

    local collective = mathx.clamp(
        tiltCompensatedCollectiveOut,
        self.collectiveMin,
        self.collectiveMax
    )

    return {
        commands = {
            collective = collective,
            roll = rollCmd,
            pitch = pitchCmd,
            yaw = yawCmd,
        },

        terms = {
            height = {
                target = verticalTarget.down,
                current = pose.down,
                err = heightErr,
                out = targetDownSpeed,
                lockActive = verticalTarget.active,
                lockPending = verticalTarget.pending,
            },

            verticalSpeed = {
                target = targetDownSpeed,
                current = downSpeed,
                err = downSpeedErr,
                feedforward = collectiveFeedforward,
                feedback = collectiveFeedback,
                controlFeedforward = downSpeedFeedforward,
                controlFeedback = downSpeedFeedback,
                tiltCompensation = tiltCompensation,
                tiltVerticalFactor = tiltVerticalFactor,
                uncompensatedOut = collectiveOut,
                out = tiltCompensatedCollectiveOut,
            },

            roll = {
                target = attitudeTarget.roll,
                current = pose.roll,
                err = rollErr,
                rate = rollRate,
                out = rollCmd,
            },

            pitch = {
                target = attitudeTarget.pitch,
                current = pose.pitch,
                err = pitchErr,
                rate = pitchRate,
                feedforward = pitchFeedforward,
                feedback = pitchFeedback,
                out = pitchCmd,
            },

            yaw = {
                target = yawTarget.angle,
                current = pose.yaw,
                err = yawErr,
                targetRate = targetYawRate,
                rate = yawRate,
                rateErr = yawRateErr,
                rateFeedforward = yawRateFeedforward,
                rateFeedback = yawRateFeedback,
                out = yawCmd,
                angleActive = yawAngleActive,
                anglePending = yawTarget.pending,
            },
        },
    }
end

function Controller:pidControllers()
    return {
        height = self.height,
        verticalSpeed = self.verticalSpeed,
        roll = self.roll,
        pitch = self.pitch,
        yawAngle = self.yawAngle,
        yawRate = self.yawRate,
    }
end

return controller
