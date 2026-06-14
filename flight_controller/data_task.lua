local mathx = require("lib.mathx")
local config = require("config")

local data_task = {}

local BODY_AXIS = config.calibration.body_axis or {
    forward = { x = 0, y = 0, z = 1 },
    right = { x = 1, y = 0, z = 0 },
    down = { x = 0, y = -1, z = 0 },
}
local SENSOR_AXIS = config.calibration.sensor_axis
local RUNTIME_DATA = config.runtime.data

local LINEAR_VELOCITY_DT = RUNTIME_DATA.linear_velocity_dt

local function axisVector(axis)
    return vector.new(axis.x or 0.0, axis.y or 0.0, axis.z or 0.0)
end

local BODY_FORWARD = axisVector(BODY_AXIS.forward)
local BODY_RIGHT = axisVector(BODY_AXIS.right)
local BODY_DOWN = axisVector(BODY_AXIS.down)

local function dot(a, b)
    return a.x * b.x + a.y * b.y + a.z * b.z
end

local function horizontalWorldToBody(x, z, yaw)
    return {
        right = math.cos(yaw) * x + math.sin(yaw) * z,
        forward = math.sin(yaw) * x - math.cos(yaw) * z,
    }
end

local function rawPositionFromVector(v)
    return {
        x = v.x,
        y = v.y,
        z = v.z,
    }
end

local function navigationPointFromRawPosition(position)
    local targetX = position.x
    local targetZ = position.z

    return {
        frdErrorFrom = function(_, currentPosition, yaw)
            return horizontalWorldToBody(
                targetX - currentPosition.x,
                targetZ - currentPosition.z,
                yaw
            )
        end,
    }
end

local function velocityFromVector(v, yaw)
    local x = v.x
    local y = v.y
    local z = v.z
    local horizontal = horizontalWorldToBody(x, z, yaw)
    local total = math.sqrt(x * x + y * y + z * z)
    local horizontalSpeed = math.sqrt(x * x + z * z)

    return {
        forward = horizontal.forward,
        right = horizontal.right,
        down = -y,
        total = total,
        horizontal = horizontalSpeed,
    }, {
        x = x,
        y = y,
        z = z,
        total = total,
        horizontal = horizontalSpeed,
        vertical = y,
    }
end

local function angularRatesFromVector(v, basis)
    return {
        roll = SENSOR_AXIS.roll * dot(v, basis.forward),
        pitch = -SENSOR_AXIS.pitch * dot(v, basis.right),
        yaw = SENSOR_AXIS.yaw * dot(v, basis.down),
    }
end

local function getPose()
    local rawPose = sublevel.getLogicalPose()
    local q = rawPose.orientation:normalize()

    local forward = q:mul(BODY_FORWARD)
    local right = q:mul(BODY_RIGHT)
    local down = q:mul(BODY_DOWN)
    local rawPosition = rawPositionFromVector(rawPose.position)

    local horizontal = math.sqrt(forward.x * forward.x + forward.z * forward.z)

    local pitch = mathx.atan2(-forward.y, horizontal)
    local roll = mathx.atan2(-right.y, -down.y)
    local yaw = mathx.atan2(forward.x, -forward.z)

    local controlPose = {
        down = -rawPosition.y,
        roll = mathx.wrapPi(SENSOR_AXIS.roll * roll),
        pitch = mathx.wrapPi(SENSOR_AXIS.pitch * pitch),
        yaw = mathx.wrapPi(SENSOR_AXIS.yaw * yaw),
    }

    function controlPose:captureNavigationPoint()
        return navigationPointFromRawPosition(rawPosition)
    end

    function controlPose:frdErrorToNavigationPoint(target)
        return target:frdErrorFrom(rawPosition, self.yaw)
    end

    return controlPose, rawPosition, {
        forward = forward,
        right = right,
        down = down,
    }
end

local function waitForPose(shared)
    while shared.running and shared.pose == nil do
        sleep(0)
    end
end

function data_task.run(shared)
    local latestBasis = nil

    local function poseTask()
        while shared.running do
            local pose, rawPosition, basis = getPose()

            latestBasis = basis
            shared.pose = pose
            shared.rawPosition = rawPosition
            shared.poseTime = os.clock()
            sleep(0)
        end
    end

    local function angularVelocityTask()
        waitForPose(shared)

        while shared.running do
            local basis = latestBasis

            if basis ~= nil then
                local angularVelocity = sublevel.getAngularVelocity()
                local rates = angularRatesFromVector(angularVelocity, basis)
                local now = os.clock()

                shared.rollRate = rates.roll
                shared.pitchRate = rates.pitch
                shared.yawRate = rates.yaw
                shared.rollRateTime = now
                shared.pitchRateTime = now
                shared.yawRateTime = now
            end

            sleep(0)
        end
    end

    local function linearVelocityTask()
        waitForPose(shared)

        while shared.running do
            local velocity, rawVelocity = velocityFromVector(
                sublevel.getLinearVelocity(),
                shared.pose.yaw
            )

            shared.velocity = velocity
            shared.rawVelocity = rawVelocity
            shared.velocityTime = os.clock()

            sleep(LINEAR_VELOCITY_DT)
        end
    end

    parallel.waitForAny(poseTask, angularVelocityTask, linearVelocityTask)
end

return data_task
