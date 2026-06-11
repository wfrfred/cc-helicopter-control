local mathx = require("lib.mathx")
local quat = require("quat")
local config = require("config")

local data_task = {}

local BODY_FRONT = vector.new(0, 0, 1)
local BODY_RIGHT = vector.new(1, 0, 0)
local BODY_UP = vector.new(0, 1, 0)

local ROLL_SIGN = config.data.roll_sign
local PITCH_SIGN = config.data.pitch_sign
local YAW_SIGN = config.data.yaw_sign

local YAW_RATE_SIGN = config.data.yaw_rate_sign

local function dot(a, b)
    return a.x * b.x + a.y * b.y + a.z * b.z
end

local function getState()
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

        roll = mathx.wrapPi(ROLL_SIGN * roll),
        pitch = mathx.wrapPi(PITCH_SIGN * pitch),
        yaw = mathx.wrapPi(YAW_SIGN * yaw),
    }
end

function data_task.run(shared)
    local function poseTask()
        while shared.running do
            local ok, result = pcall(getState)

            if ok then
                shared.state = result
                shared.stateTime = os.clock()
                shared.lastError = nil
            else
                shared.lastError = result
            end

            sleep(0)
        end
    end

    local function angularVelocityTask()
        while shared.running do
            local ok, result = pcall(sublevel.getAngularVelocity)

            if ok then
                local s = shared.state

                if s then
                    shared.yawRate = YAW_RATE_SIGN * YAW_SIGN * dot(result, s.up)
                    shared.yawRateTime = os.clock()
                end
            else
                shared.lastError = result
            end

            sleep(0)
        end
    end

    parallel.waitForAny(poseTask, angularVelocityTask)
end

return data_task
