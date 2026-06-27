local frames = require("lib.frames")

local control_state = {}

local function express(worldPhysical, targetFrame)
    return {
        position = targetFrame:coordinatesOf(worldPhysical.position),
        orientation = targetFrame:localOrientationOf(worldPhysical.orientation),
        velocity = targetFrame:componentsOf(worldPhysical.velocity),
        angularVelocity = targetFrame:componentsOf(worldPhysical.angularVelocity),
    }
end

function control_state.ready(samples)
    return samples ~= nil
        and samples.pose ~= nil
        and samples.pose.raw ~= nil
        and samples.pose.raw.position ~= nil
        and samples.pose.raw.orientation ~= nil
        and samples.pose.time ~= nil
        and samples.velocity ~= nil
        and samples.velocity.world ~= nil
        and samples.velocity.time ~= nil
        and samples.angularVelocity ~= nil
        and samples.angularVelocity.raw ~= nil
        and samples.angularVelocity.time ~= nil
end

function control_state.fromSensors(samples, options)
    options = options or {}
    assert(options.bodyAxis ~= nil, "control_state.fromSensors requires bodyAxis")

    local pose = samples.pose
    local velocity = samples.velocity
    local angular = samples.angularVelocity
    local rawPose = pose.raw
    local worldFrame = frames.world()
    local bodyFrame = frames.body(rawPose, options.bodyAxis)
    local navigationFrame = frames.navigation(bodyFrame)
    local bodyAngularVelocity = frames.bodyAngularVector(angular.raw, options.bodyAxis)
    local world = {
        position = rawPose.position,
        orientation = bodyFrame.qWorldFromLocal,
        velocity = velocity.world,
        angularVelocity = bodyFrame:vector(bodyAngularVelocity),
    }

    return {
        frames = {
            world = worldFrame,
            navigation = navigationFrame,
            body = bodyFrame,
        },
        world = world,
        navigation = express(world, navigationFrame),
        body = express(world, bodyFrame),
        sampleTime = {
            pose = pose.time,
            velocity = velocity.time,
            angularVelocity = angular.time,
        },
    }
end

return control_state
