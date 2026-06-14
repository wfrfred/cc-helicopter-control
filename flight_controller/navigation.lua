local navigation = {}

local function projectWorldHorizontalToBodyFrd(x, z, yaw)
    return {
        right = math.cos(yaw) * x + math.sin(yaw) * z,
        forward = math.sin(yaw) * x - math.cos(yaw) * z,
    }
end

function navigation.makePositionTarget(state)
    local position = state.raw.position

    return {
        x = position.x,
        z = position.z,
    }
end

function navigation.projectPositionTargetErrorToBodyFrd(target, state)
    local position = state.raw.position

    return projectWorldHorizontalToBodyFrd(
        target.x - position.x,
        target.z - position.z,
        state.body.pose.yaw
    )
end

return navigation
