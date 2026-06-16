local mathx = require("lib.mathx")
local config = require("config")

local data_task = {}

local bodyAxis = config.calibration.body_axis

local function buildBodyFrame(rawPose)
    local q = rawPose.orientation:normalize()

    return {
        forward = q:mul(bodyAxis.forward),
        right = q:mul(bodyAxis.right),
        down = q:mul(bodyAxis.down),
    }
end

local function buildPose(rawPosition, bodyFrame)
    local horizontal = math.sqrt(
        bodyFrame.forward.x * bodyFrame.forward.x
            + bodyFrame.forward.z * bodyFrame.forward.z
    )
    local roll = mathx.atan2(-bodyFrame.right.y, -bodyFrame.down.y)
    local pitch = mathx.atan2(bodyFrame.forward.y, horizontal)
    local heading = mathx.atan2(bodyFrame.forward.x, -bodyFrame.forward.z)

    return {
        height = rawPosition.y,
        roll = mathx.wrapPi(roll),
        pitch = mathx.wrapPi(pitch),
        heading = mathx.wrapPi(heading),
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
    local bodyFrame = buildBodyFrame(rawPose)

    return {
        raw = {
            position = rawPose.position,
            orientation = rawPose.orientation,
        },
        body = {
            frame = bodyFrame,
            pose = buildPose(rawPose.position, bodyFrame),
        },
        time = os.clock(),
    }
end

local function readLinearVelocity(bodyFrame)
    local worldVelocity = sublevel.getLinearVelocity()

    return {
        raw = {
            velocity = worldVelocity,
        },
        body = {
            velocity = mathx.project(worldVelocity, {
                forward = bodyFrame.forward,
                right = bodyFrame.right,
                down = bodyFrame.down,
            }),
        },
        time = os.clock(),
    }
end

local function readRates()
    local rawAngularVelocity = sublevel.getAngularVelocity()

    return {
        raw = {
            angularVelocity = rawAngularVelocity,
        },
        body = {
            rates = mathx.project(rawAngularVelocity, {
                roll = bodyAxis.forward,
                pitch = bodyAxis.right,
                yaw = bodyAxis.down,
            }),
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

    local latestBodyFrame = nil

    local function poseTask()
        while shared.running do
            local pose = readPose()
            latestBodyFrame = pose.body.frame

            shared.state.raw.position = pose.raw.position
            shared.state.raw.orientation = pose.raw.orientation
            shared.state.body.frame = pose.body.frame
            shared.state.body.pose = pose.body.pose
            shared.state.time.pose = pose.time

            sleep(0)
        end
    end

    local function angularVelocityTask()
        waitForPose(shared)

        while shared.running and latestBodyFrame == nil do
            sleep(0)
        end

        while shared.running do
            local rates = readRates()

            shared.state.raw.angularVelocity = rates.raw.angularVelocity
            shared.state.body.rates = rates.body.rates
            shared.state.time.rates = rates.time

            sleep(0)
        end
    end

    local function linearVelocityTask()
        waitForPose(shared)

        while shared.running and latestBodyFrame == nil do
            sleep(0)
        end

        while shared.running do
            local velocity = readLinearVelocity(latestBodyFrame)

            shared.state.raw.velocity = velocity.raw.velocity
            shared.state.body.velocity = velocity.body.velocity
            shared.state.time.velocity = velocity.time

            sleep(0)
        end
    end

    parallel.waitForAny(poseTask, angularVelocityTask, linearVelocityTask)
end

return data_task
