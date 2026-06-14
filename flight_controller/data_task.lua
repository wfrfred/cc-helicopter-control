local mathx = require("lib.mathx")
local config = require("config")

local data_task = {}

local BODY_AXIS = config.calibration.body_axis
local RUNTIME_DATA = config.runtime.data

local LINEAR_VELOCITY_DT = RUNTIME_DATA.linear_velocity_dt

local function readRawPose()
    return sublevel.getLogicalPose()
end

local function readRawVelocity()
    return sublevel.getLinearVelocity()
end

local function readRawAngularVelocity()
    return sublevel.getAngularVelocity()
end

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

local function projectAngularVelocityToBodyRates(rawAngularVelocity, bodyFrame)
    return {
        roll = rawAngularVelocity:dot(bodyFrame.forward),
        pitch = rawAngularVelocity:dot(bodyFrame.right),
        yaw = rawAngularVelocity:dot(bodyFrame.down),
    }
end

local function buildBodyFrame(rawPose)
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

local function buildBodyPose(rawPosition, bodyFrame)
    local horizontal = math.sqrt(
        bodyFrame.forward.x * bodyFrame.forward.x
            + bodyFrame.forward.z * bodyFrame.forward.z
    )
    local roll = mathx.atan2(-bodyFrame.right.y, -bodyFrame.down.y)
    local pitch = mathx.atan2(-bodyFrame.forward.y, horizontal)
    local yaw = mathx.atan2(bodyFrame.forward.x, -bodyFrame.forward.z)

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

local function makeRawVelocity(rawVelocity)
    return {
        x = rawVelocity.x,
        y = rawVelocity.y,
        z = rawVelocity.z,
        total = rawVelocity:length(),
        horizontal = math.sqrt(rawVelocity.x * rawVelocity.x + rawVelocity.z * rawVelocity.z),
        vertical = rawVelocity.y,
    }
end

local function buildPoseSnapshot(rawPose)
    local snapshot = {
        raw = {
            pose = rawPose,
            position = rawPose.position,
        },
        body = {},
    }

    snapshot.body.frame = buildBodyFrame(snapshot.raw.pose)
    snapshot.body.pose = buildBodyPose(snapshot.raw.position, snapshot.body.frame)

    return snapshot
end

local function buildVelocitySnapshot(rawVelocity, yaw)
    local snapshot = {
        raw = {
            velocity = makeRawVelocity(rawVelocity),
        },
        body = {},
    }

    snapshot.body.velocity = projectVelocityToBodyFrd(snapshot.raw.velocity, yaw)

    return snapshot
end

local function buildAngularVelocitySnapshot(rawAngularVelocity, bodyFrame)
    return {
        raw = {
            angularVelocity = rawAngularVelocity,
        },
        body = {
            rates = projectAngularVelocityToBodyRates(rawAngularVelocity, bodyFrame),
        },
    }
end

local function publishPoseSnapshot(shared, snapshot)
    shared.pose = snapshot.body.pose
    shared.rawPosition = snapshot.raw.position
    shared.poseTime = os.clock()
end

local function publishVelocitySnapshot(shared, snapshot)
    shared.velocity = snapshot.body.velocity
    shared.rawVelocity = snapshot.raw.velocity
    shared.velocityTime = os.clock()
end

local function publishAngularVelocitySnapshot(shared, snapshot)
    local now = os.clock()

    shared.rollRate = snapshot.body.rates.roll
    shared.pitchRate = snapshot.body.rates.pitch
    shared.yawRate = snapshot.body.rates.yaw
    shared.rollRateTime = now
    shared.pitchRateTime = now
    shared.yawRateTime = now
end

local function waitForPose(shared)
    while shared.running and shared.pose == nil do
        sleep(0)
    end
end

function data_task.run(shared)
    local latestPoseSnapshot = nil

    local function poseTask()
        while shared.running do
            latestPoseSnapshot = buildPoseSnapshot(readRawPose())
            publishPoseSnapshot(shared, latestPoseSnapshot)
            sleep(0)
        end
    end

    local function angularVelocityTask()
        waitForPose(shared)

        while shared.running and latestPoseSnapshot == nil do
            sleep(0)
        end

        while shared.running do
            publishAngularVelocitySnapshot(
                shared,
                buildAngularVelocitySnapshot(readRawAngularVelocity(), latestPoseSnapshot.body.frame)
            )
            sleep(0)
        end
    end

    local function linearVelocityTask()
        waitForPose(shared)

        while shared.running do
            publishVelocitySnapshot(shared, buildVelocitySnapshot(readRawVelocity(), shared.pose.yaw))
            sleep(LINEAR_VELOCITY_DT)
        end
    end

    parallel.waitForAny(poseTask, angularVelocityTask, linearVelocityTask)
end

return data_task
