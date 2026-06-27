local mathx = require("lib.mathx")
local config = require("config")
local frames = require("lib.frames")

local sensor_task = {}

local bodyAxis = config.calibration.body_axis

local function buildPose(rawPosition, bodyFrame)
    local basis = bodyFrame:basis()
    local forwardHorizontal = vector.new(basis.forward.x, 0.0, basis.forward.z)
    local horizontal = forwardHorizontal:length()
    local roll = mathx.atan2(-basis.right.y, -basis.down.y)
    local pitch = mathx.atan2(basis.forward.y, horizontal)
    local heading = mathx.atan2(basis.forward.x, -basis.forward.z)

    return {
        height = rawPosition.y,
        roll = mathx.wrapPi(roll),
        pitch = mathx.wrapPi(pitch),
        heading = mathx.wrapPi(heading),
    }
end

function sensor_task.headingRateFromAngular(bodyFrame, angular)
    local basis = bodyFrame:basis()
    local forward = basis.forward
    local forwardHorizontal = vector.new(forward.x, 0.0, forward.z)
    local horizontal = forwardHorizontal:dot(forwardHorizontal)

    if horizontal < 1.0e-6 then
        return 0.0
    end

    local function fromForwardChange(x, z)
        return (-forward.z * x + forward.x * z) / horizontal
    end

    return (angular.pitch or 0.0) * fromForwardChange(-basis.down.x, -basis.down.z)
        + (angular.yaw or 0.0) * fromForwardChange(basis.right.x, basis.right.z)
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
    local bodyFrame = frames.bodyFromPose(rawPose, bodyAxis)
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
    local worldVelocity = rawVelocity
    local bodyVelocity = frames.frdFromVector(bodyFrame:componentsOf(worldVelocity))
    local navigationFrd = frames.frdFromVector(
        frames.level(heading):componentsOf(worldVelocity)
    )

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
            velocity = {
                forward = navigationFrd.forward,
                right = navigationFrd.right,
                up = -navigationFrd.down,
            },
        },
        time = os.clock(),
    }
end

local function readAngularVelocity(bodyFrame)
    local rawAngularVelocity = sublevel.getAngularVelocity()
    local bodyAngularVelocity = bodyFrame:componentsOf(rawAngularVelocity)
    local angular = {
        roll = bodyAngularVelocity.x,
        pitch = bodyAngularVelocity.y,
        yaw = bodyAngularVelocity.z,
    }

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
