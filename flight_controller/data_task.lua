local mathx = require("lib.mathx")
local quat = require("quat")
local config = require("config")

local data_task = {}

local BODY_FRONT = vector.new(0, 0, 1)
local BODY_RIGHT = vector.new(1, 0, 0)
local BODY_UP = vector.new(0, 1, 0)

local SENSOR_AXIS = config.calibration.sensor_axis
local RUNTIME_DATA = config.runtime.data

local LINEAR_VELOCITY_DT = RUNTIME_DATA.linear_velocity_dt

local function dot(a, b)
    return a.x * b.x + a.y * b.y + a.z * b.z
end

local function velocityFromVector(v)
    local x = v.x
    local y = v.y
    local z = v.z

    return {
        x = x,
        y = y,
        z = z,
        total = math.sqrt(x * x + y * y + z * z),
        horizontal = math.sqrt(x * x + z * z),
        vertical = y,
    }
end

local function getPose()
    local pose = sublevel.getLogicalPose()
    local q = quat.fromSable(pose.orientation)

    local front = quat.rotate(q, BODY_FRONT)
    local right = quat.rotate(q, BODY_RIGHT)
    local up = quat.rotate(q, BODY_UP)

    local horizontal = math.sqrt(front.x * front.x + front.z * front.z)

    local pitch = mathx.atan2(-front.y, horizontal)
    local roll = mathx.atan2(-right.y, up.y)
    local yaw = mathx.atan2(front.x, -front.z)

    return {
        pos = pose.position,

        front = front,
        right = right,
        up = up,

        roll = mathx.wrapPi(SENSOR_AXIS.roll * roll),
        pitch = mathx.wrapPi(SENSOR_AXIS.pitch * pitch),
        yaw = mathx.wrapPi(SENSOR_AXIS.yaw * yaw),
    }
end

local function waitForPose(shared)
    while shared.running and shared.pose == nil do
        sleep(0)
    end
end

function data_task.run(shared)
    local function poseTask()
        while shared.running do
            shared.pose = getPose()
            shared.poseTime = os.clock()
            sleep(0)
        end
    end

    local function angularVelocityTask()
        waitForPose(shared)

        while shared.running do
            local angularVelocity = sublevel.getAngularVelocity()

            shared.yawRate = SENSOR_AXIS.yaw_rate * SENSOR_AXIS.yaw * dot(angularVelocity, shared.pose.up)
            shared.yawRateTime = os.clock()
            sleep(0)
        end
    end

    local function linearVelocityTask()
        while shared.running do
            shared.velocity = velocityFromVector(sublevel.getLinearVelocity())
            shared.velocityTime = os.clock()

            sleep(LINEAR_VELOCITY_DT)
        end
    end

    parallel.waitForAny(poseTask, angularVelocityTask, linearVelocityTask)
end

return data_task
