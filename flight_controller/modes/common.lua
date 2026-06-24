local common = {}

local function zeroAxes()
    return {
        roll = 0.0,
        pitch = 0.0,
        yaw = 0.0,
    }
end

local function copyAxes(value)
    if value == nil then
        return zeroAxes()
    end

    return {
        roll = value.roll or 0.0,
        pitch = value.pitch or 0.0,
        yaw = value.yaw or 0.0,
    }
end

local function emptyTranslationPosition()
    return {
        forward = nil,
        right = nil,
        down = nil,
    }
end

local function zeroTranslation()
    return {
        forward = 0.0,
        right = 0.0,
        down = 0.0,
    }
end

local function copyTranslation(value)
    if value == nil then
        return zeroTranslation()
    end

    return {
        forward = value.forward or 0.0,
        right = value.right or 0.0,
        down = value.down or 0.0,
    }
end

local function copyTranslationPosition(value)
    if value == nil then
        return emptyTranslationPosition()
    end

    return {
        forward = value.forward,
        right = value.right,
        down = value.down,
    }
end

local function copyAttitudeAngle(value)
    value = value or {}

    return {
        roll = value.roll,
        pitch = value.pitch,
        yaw = value.yaw,
    }
end

function common.target(input)
    input = input or {}

    return {
        translation = {
            position = copyTranslationPosition(input.position),
            feedforward = copyTranslation(input.feedforward),
        },
        attitude = {
            angle = copyAttitudeAngle(input.attitude),
            feedforward = {
                angle = copyAxes(input.attitudeFeedforwardAngle),
                rate = copyAxes(input.attitudeFeedforwardRate),
            },
        },
    }
end

function common.status(active)
    return {
        active = active == true,
    }
end

function common.frdFromWorld(value, heading)
    local horizontal = vector.new(value.x or 0.0, 0.0, value.z or 0.0)
    local forward = vector.new(math.sin(heading), 0.0, -math.cos(heading))
    local right = vector.new(math.cos(heading), 0.0, math.sin(heading))

    return {
        forward = horizontal:dot(forward),
        right = horizontal:dot(right),
        down = -(value.y or 0.0),
    }
end

return common
