local mathx = require("lib.mathx")
local attitude_math = require("lib.attitude_math")
local config = require("config")

local sensor_task = {}

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
    local forwardHorizontal = vector.new(bodyFrame.forward.x, 0.0, bodyFrame.forward.z)
    local horizontal = forwardHorizontal:length()
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

function sensor_task.headingRateFromAngular(bodyFrame, angular)
    local forward = bodyFrame.forward
    local forwardHorizontal = vector.new(forward.x, 0.0, forward.z)
    local horizontal = forwardHorizontal:dot(forwardHorizontal)

    if horizontal < 1.0e-6 then
        return 0.0
    end

    local function fromForwardChange(x, z)
        return (-forward.z * x + forward.x * z) / horizontal
    end

    return (angular.pitch or 0.0) * fromForwardChange(-bodyFrame.down.x, -bodyFrame.down.z)
        + (angular.yaw or 0.0) * fromForwardChange(bodyFrame.right.x, bodyFrame.right.z)
end

function sensor_task.navigationVelocity(worldVelocity, heading)
    local frd = attitude_math.levelFrdFromWorld(worldVelocity, heading)

    return {
        forward = frd.forward,
        right = frd.right,
        up = -frd.down,
    }
end

function sensor_task.bodyVelocityFromRaw(rawVelocity)
    return mathx.project(rawVelocity, {
        forward = bodyAxis.forward,
        right = bodyAxis.right,
        down = bodyAxis.down,
    })
end

function sensor_task.worldVelocityFromBody(bodyFrame, bodyVelocity)
    return bodyFrame.forward * (bodyVelocity.forward or 0.0)
        + bodyFrame.right * (bodyVelocity.right or 0.0)
        + bodyFrame.down * (bodyVelocity.down or 0.0)
end

local function makeState()
    return {
        raw = {},
        world = {},
        body = {},
        navigation = {
            heading = {},
            velocity = {},
        },
        time = {},
    }
end

local function readPose()
    local rawPose = sublevel.getLogicalPose()
    local bodyFrame = buildBodyFrame(rawPose)
    local pose = buildPose(rawPose.position, bodyFrame)

    return {
        raw = {
            position = rawPose.position,
            orientation = rawPose.orientation,
        },
        world = {
            position = rawPose.position,
        },
        body = {
            frame = bodyFrame,
            orientation = attitude_math.quaternionFromFrame(bodyFrame),
            pose = pose,
        },
        navigation = {
            heading = {
                angle = pose.heading,
            },
        },
        time = os.clock(),
    }
end

local function readLinearVelocity(bodyFrame, heading)
    local rawVelocity = sublevel.getLinearVelocity()
    local bodyVelocity = sensor_task.bodyVelocityFromRaw(rawVelocity)
    local worldVelocity = sensor_task.worldVelocityFromBody(bodyFrame, bodyVelocity)

    return {
        raw = {
            velocity = rawVelocity,
        },
        world = {
            velocity = worldVelocity,
        },
        body = {
            velocity = bodyVelocity,
        },
        navigation = {
            velocity = sensor_task.navigationVelocity(worldVelocity, heading),
        },
        time = os.clock(),
    }
end

local function readAngularVelocity(bodyFrame)
    local rawAngularVelocity = sublevel.getAngularVelocity()
    local angular = mathx.project(rawAngularVelocity, {
        roll = bodyAxis.forward,
        pitch = bodyAxis.right,
        yaw = bodyAxis.down,
    })

    return {
        raw = {
            angularVelocity = rawAngularVelocity,
        },
        body = {
            angular = {
                velocity = angular,
            },
        },
        navigation = {
            heading = {
                rate = sensor_task.headingRateFromAngular(bodyFrame, angular),
            },
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

function sensor_task.run(shared)
    shared.state = makeState()

    local latestBodyFrame = nil

    local function poseTask()
        while shared.running do
            local pose = readPose()
            latestBodyFrame = pose.body.frame

            shared.state.raw.position = pose.raw.position
            shared.state.raw.orientation = pose.raw.orientation
            shared.state.world.position = pose.world.position
            shared.state.body.frame = pose.body.frame
            shared.state.body.orientation = pose.body.orientation
            shared.state.body.pose = pose.body.pose
            shared.state.navigation.heading.angle = pose.navigation.heading.angle
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
            local rates = readAngularVelocity(latestBodyFrame)

            shared.state.raw.angularVelocity = rates.raw.angularVelocity
            shared.state.body.angular = rates.body.angular
            shared.state.navigation.heading.rate = rates.navigation.heading.rate
            shared.state.time.angularVelocity = rates.time

            sleep(0)
        end
    end

    local function linearVelocityTask()
        waitForPose(shared)

        while shared.running and latestBodyFrame == nil do
            sleep(0)
        end

        while shared.running do
            local velocity = readLinearVelocity(latestBodyFrame, shared.state.navigation.heading.angle)

            shared.state.raw.velocity = velocity.raw.velocity
            shared.state.world.velocity = velocity.world.velocity
            shared.state.body.velocity = velocity.body.velocity
            shared.state.navigation.velocity = velocity.navigation.velocity
            shared.state.time.velocity = velocity.time

            sleep(0)
        end
    end

    parallel.waitForAny(poseTask, angularVelocityTask, linearVelocityTask)
end

return sensor_task
