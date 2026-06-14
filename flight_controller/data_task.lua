local mathx = require("lib.mathx")
local config = require("config")

local data_task = {}

local BODY_AXIS = config.calibration.body_axis
local RUNTIME_DATA = config.runtime.data

local LINEAR_VELOCITY_DT = RUNTIME_DATA.linear_velocity_dt

local function worldHorizontalToBodyFrd(x, z, yaw)
    return {
        right = math.cos(yaw) * x + math.sin(yaw) * z,
        forward = math.sin(yaw) * x - math.cos(yaw) * z,
    }
end

local function navigationPointFromRawPosition(position)
    return {
        frdErrorFrom = function(currentPosition, yaw)
            return worldHorizontalToBodyFrd(
                position.x - currentPosition.x,
                position.z - currentPosition.z,
                yaw
            )
        end,
    }
end

local function bodyFrameFromOrientation(orientation)
    local q = orientation:normalize()

    return {
        forward = q:mul(BODY_AXIS.forward),
        right = q:mul(BODY_AXIS.right),
        down = q:mul(BODY_AXIS.down),
    }
end

local function bodyPoseFromSample(sample)
    local horizontal = math.sqrt(
        sample.body.frame.forward.x * sample.body.frame.forward.x
            + sample.body.frame.forward.z * sample.body.frame.forward.z
    )
    local roll = mathx.atan2(-sample.body.frame.right.y, -sample.body.frame.down.y)
    local pitch = mathx.atan2(-sample.body.frame.forward.y, horizontal)
    local yaw = mathx.atan2(sample.body.frame.forward.x, -sample.body.frame.forward.z)

    local pose = {
        down = -sample.raw.position.y,
        roll = mathx.wrapPi(-roll),
        pitch = mathx.wrapPi(pitch),
        yaw = mathx.wrapPi(yaw),
    }

    function pose:captureNavigationPoint()
        return navigationPointFromRawPosition(sample.raw.position)
    end

    function pose:frdErrorToNavigationPoint(target)
        return target.frdErrorFrom(sample.raw.position, self.yaw)
    end

    return pose
end

local function poseSample()
    local rawPose = sublevel.getLogicalPose()
    local sample = {
        raw = {
            position = rawPose.position,
        },
        body = {
            frame = bodyFrameFromOrientation(rawPose.orientation),
        },
    }

    sample.body.pose = bodyPoseFromSample(sample)

    return sample
end

local function velocitySample(yaw)
    local rawVelocity = sublevel.getLinearVelocity()
    local bodyHorizontal = worldHorizontalToBodyFrd(rawVelocity.x, rawVelocity.z, yaw)
    local totalSpeed = rawVelocity:length()
    local horizontalSpeed = math.sqrt(rawVelocity.x * rawVelocity.x + rawVelocity.z * rawVelocity.z)

    return {
        raw = {
            velocity = {
                x = rawVelocity.x,
                y = rawVelocity.y,
                z = rawVelocity.z,
                total = totalSpeed,
                horizontal = horizontalSpeed,
                vertical = rawVelocity.y,
            },
        },
        body = {
            velocity = {
                forward = bodyHorizontal.forward,
                right = bodyHorizontal.right,
                down = -rawVelocity.y,
                total = totalSpeed,
                horizontal = horizontalSpeed,
            },
        },
    }
end

local function angularVelocitySample(bodyFrame)
    local rawAngularVelocity = sublevel.getAngularVelocity()

    return {
        raw = {
            angularVelocity = rawAngularVelocity,
        },
        body = {
            rates = {
                roll = -rawAngularVelocity:dot(bodyFrame.forward),
                pitch = -rawAngularVelocity:dot(bodyFrame.right),
                yaw = rawAngularVelocity:dot(bodyFrame.down),
            },
        },
    }
end

local function waitForPose(shared)
    while shared.running and shared.pose == nil do
        sleep(0)
    end
end

function data_task.run(shared)
    local latestPoseSample = nil

    local function poseTask()
        while shared.running do
            latestPoseSample = poseSample()

            shared.pose = latestPoseSample.body.pose
            shared.rawPosition = latestPoseSample.raw.position
            shared.poseTime = os.clock()
            sleep(0)
        end
    end

    local function angularVelocityTask()
        waitForPose(shared)

        while shared.running and latestPoseSample == nil do
            sleep(0)
        end

        while shared.running do
            local sample = angularVelocitySample(latestPoseSample.body.frame)
            local now = os.clock()

            shared.rollRate = sample.body.rates.roll
            shared.pitchRate = sample.body.rates.pitch
            shared.yawRate = sample.body.rates.yaw
            shared.rollRateTime = now
            shared.pitchRateTime = now
            shared.yawRateTime = now

            sleep(0)
        end
    end

    local function linearVelocityTask()
        waitForPose(shared)

        while shared.running do
            local sample = velocitySample(shared.pose.yaw)

            shared.velocity = sample.body.velocity
            shared.rawVelocity = sample.raw.velocity
            shared.velocityTime = os.clock()

            sleep(LINEAR_VELOCITY_DT)
        end
    end

    parallel.waitForAny(poseTask, angularVelocityTask, linearVelocityTask)
end

return data_task
