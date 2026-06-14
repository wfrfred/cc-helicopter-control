local mathx = require("lib.mathx")
local config = require("config")

local data_task = {}

local BODY_AXIS = config.calibration.body_axis
local RUNTIME_DATA = config.runtime.data

local LINEAR_VELOCITY_DT = RUNTIME_DATA.linear_velocity_dt

local function projectWorldHorizontalToBodyFrd(x, z, yaw)
    return {
        right = math.cos(yaw) * x + math.sin(yaw) * z,
        forward = math.sin(yaw) * x - math.cos(yaw) * z,
    }
end

local function projectVelocityToBodyFrd(rawVelocity, yaw)
    local horizontal = projectWorldHorizontalToBodyFrd(rawVelocity.x, rawVelocity.z, yaw)

    return {
        forward = horizontal.forward,
        right = horizontal.right,
        down = -rawVelocity.y,
        total = rawVelocity.total,
        horizontal = rawVelocity.horizontal,
    }
end

local function projectAngularVelocityToBodyRates(rawAngularVelocity, frame)
    return {
        roll = rawAngularVelocity:dot(frame.forward),
        pitch = rawAngularVelocity:dot(frame.right),
        yaw = rawAngularVelocity:dot(frame.down),
    }
end

local function buildFrame(rawPose)
    local q = rawPose.orientation:normalize()

    return {
        forward = q:mul(BODY_AXIS.forward),
        right = q:mul(BODY_AXIS.right),
        down = q:mul(BODY_AXIS.down),
    }
end

local function makeNavigationPoint(rawPosition)
    return {
        projectErrorToBodyFrd = function(currentPosition, yaw)
            return projectWorldHorizontalToBodyFrd(
                rawPosition.x - currentPosition.x,
                rawPosition.z - currentPosition.z,
                yaw
            )
        end,
    }
end

local function buildPose(rawPosition, frame)
    local horizontal = math.sqrt(
        frame.forward.x * frame.forward.x
            + frame.forward.z * frame.forward.z
    )
    local roll = mathx.atan2(-frame.right.y, -frame.down.y)
    local pitch = mathx.atan2(-frame.forward.y, horizontal)
    local yaw = mathx.atan2(frame.forward.x, -frame.forward.z)

    local pose = {
        down = -rawPosition.y,
        roll = mathx.wrapPi(roll),
        pitch = mathx.wrapPi(pitch),
        yaw = mathx.wrapPi(yaw),
    }

    function pose:captureNavigationPoint()
        return makeNavigationPoint(rawPosition)
    end

    function pose:frdErrorToNavigationPoint(target)
        return target.projectErrorToBodyFrd(rawPosition, self.yaw)
    end

    return pose
end

local function makeRawVelocity(v)
    return {
        x = v.x,
        y = v.y,
        z = v.z,
        total = v:length(),
        horizontal = math.sqrt(v.x * v.x + v.z * v.z),
        vertical = v.y,
    }
end

local function readPose()
    local rawPose = sublevel.getLogicalPose()
    local frame = buildFrame(rawPose)

    return {
        raw = {
            pose = rawPose,
            position = rawPose.position,
        },
        body = {
            frame = frame,
            pose = buildPose(rawPose.position, frame),
        },
        time = os.clock(),
    }
end

local function readVelocity(yaw)
    local rawVelocity = makeRawVelocity(sublevel.getLinearVelocity())

    return {
        raw = {
            velocity = rawVelocity,
        },
        body = {
            velocity = projectVelocityToBodyFrd(rawVelocity, yaw),
        },
        time = os.clock(),
    }
end

local function readRates(frame)
    local rawAngularVelocity = sublevel.getAngularVelocity()

    return {
        raw = {
            angularVelocity = rawAngularVelocity,
        },
        body = {
            rates = projectAngularVelocityToBodyRates(rawAngularVelocity, frame),
        },
        time = os.clock(),
    }
end

local function waitForPose(shared)
    while shared.running and shared.poseSnapshot == nil do
        sleep(0)
    end
end

function data_task.run(shared)
    local latestPoseSnapshot = nil

    local function poseTask()
        while shared.running do
            latestPoseSnapshot = readPose()
            shared.poseSnapshot = latestPoseSnapshot
            sleep(0)
        end
    end

    local function angularVelocityTask()
        waitForPose(shared)

        while shared.running and latestPoseSnapshot == nil do
            sleep(0)
        end

        while shared.running do
            shared.ratesSnapshot = readRates(latestPoseSnapshot.body.frame)
            sleep(0)
        end
    end

    local function linearVelocityTask()
        waitForPose(shared)

        while shared.running do
            shared.velocitySnapshot = readVelocity(shared.poseSnapshot.body.pose.yaw)
            sleep(LINEAR_VELOCITY_DT)
        end
    end

    parallel.waitForAny(poseTask, angularVelocityTask, linearVelocityTask)
end

return data_task
