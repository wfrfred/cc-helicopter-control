local mathx = require("lib.mathx")
local config = require("config")

local data_task = {}

local bodyAxis = config.calibration.body_axis

local function projectVelocityToBodyFrd(rawVelocity, frame)
    return {
        forward = rawVelocity:dot(frame.forward),
        right = rawVelocity:dot(frame.right),
        down = rawVelocity:dot(frame.down),
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
        forward = q:mul(bodyAxis.forward),
        right = q:mul(bodyAxis.right),
        down = q:mul(bodyAxis.down),
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

    return {
        height = rawPosition.y,
        roll = mathx.wrapPi(roll),
        pitch = mathx.wrapPi(pitch),
        yaw = mathx.wrapPi(yaw),
    }
end

local function makeState()
    return {
        raw = {},
        body = {},
        time = {},
    }
end

local function readPose()
    local rawPose = sublevel.getLogicalPose()
    local frame = buildFrame(rawPose)

    return {
        raw = {
            position = rawPose.position,
            orientation = rawPose.orientation,
        },
        body = {
            frame = frame,
            pose = buildPose(rawPose.position, frame),
        },
        time = os.clock(),
    }
end

local function readVelocity(frame)
    local rawVelocity = sublevel.getLinearVelocity()

    return {
        raw = {
            velocity = rawVelocity,
        },
        body = {
            velocity = projectVelocityToBodyFrd(rawVelocity, frame),
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
    while shared.running and (
        shared.state == nil or
        shared.state.body.pose == nil
    ) do
        sleep(0)
    end
end

function data_task.run(shared)
    shared.state = makeState()

    local latestFrame = nil

    local function poseTask()
        while shared.running do
            local pose = readPose()
            latestFrame = pose.body.frame

            shared.state.raw.position = pose.raw.position
            shared.state.raw.orientation = pose.raw.orientation
            shared.state.body.pose = pose.body.pose
            shared.state.time.pose = pose.time

            sleep(0)
        end
    end

    local function angularVelocityTask()
        waitForPose(shared)

        while shared.running and latestFrame == nil do
            sleep(0)
        end

        while shared.running do
            local rates = readRates(latestFrame)

            shared.state.raw.angularVelocity = rates.raw.angularVelocity
            shared.state.body.rates = rates.body.rates
            shared.state.time.rates = rates.time

            sleep(0)
        end
    end

    local function linearVelocityTask()
        waitForPose(shared)

        while shared.running and latestFrame == nil do
            sleep(0)
        end

        while shared.running do
            local velocity = readVelocity(latestFrame)

            shared.state.raw.velocity = velocity.raw.velocity
            shared.state.body.velocity = velocity.body.velocity
            shared.state.time.velocity = velocity.time

            sleep(config.runtime.data.linear_velocity_dt)
        end
    end

    parallel.waitForAny(poseTask, angularVelocityTask, linearVelocityTask)
end

return data_task
