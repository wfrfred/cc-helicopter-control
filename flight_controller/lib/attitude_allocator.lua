local attitude_allocator = {}

local AXIS_INDEX = {
    roll = 1,
    pitch = 2,
    yaw = 3,
}

local AXES = { "roll", "pitch", "yaw" }

local function copyCommands(commands)
    return {
        collective = commands.collective,
        roll = commands.roll,
        pitch = commands.pitch,
        yaw = commands.yaw,
    }
end

local function attitudeCommands(commands)
    return {
        roll = commands.roll,
        pitch = commands.pitch,
        yaw = commands.yaw,
    }
end

local function axisIndex(axis)
    local index = AXIS_INDEX[axis]

    assert(index ~= nil, "unknown attitude allocator axis: " .. tostring(axis))

    return index
end

local function matrixValue(matrix, row, col)
    assert(type(matrix[row]) == "table", "attitude allocator matrix row must be table")
    assert(type(matrix[row][col]) == "number", "attitude allocator matrix value must be number")

    return matrix[row][col]
end

local function copyMatrix(matrix)
    local out = {}

    assert(type(matrix) == "table", "attitude allocator matrix must be table")

    for row = 1, 3 do
        out[row] = {}

        for col = 1, 3 do
            out[row][col] = matrixValue(matrix, row, col)
        end
    end

    return out
end

local function multiplyMatrixCommands(matrix, commands)
    local input = { commands.roll, commands.pitch, commands.yaw }
    local output = {}

    for row = 1, 3 do
        output[row] = matrix[row][1] * input[1]
            + matrix[row][2] * input[2]
            + matrix[row][3] * input[3]
    end

    return {
        roll = output[1],
        pitch = output[2],
        yaw = output[3],
    }
end

local function attitudeSignal(config, pose, name)
    assert(type(pose) == "table", "attitude allocator pose must be table")

    local limitDeg = config.attitude_limit_deg or {}

    if name == "roll" then
        return math.max(-1.0, math.min(1.0, pose.roll / math.rad(limitDeg.roll or 30.0)))
    end

    if name == "pitch" then
        return math.max(-1.0, math.min(1.0, pose.pitch / math.rad(limitDeg.pitch or 25.0)))
    end

    error("unknown attitude allocator attitude: " .. tostring(name))
end

local function scheduleMatrix(config, pose)
    local matrix = copyMatrix(config.base_matrix)

    for _, term in ipairs(config.terms or {}) do
        local row = axisIndex(term.out)
        local col = axisIndex(term.input)
        local attitude = attitudeSignal(config, pose, term.attitude)

        assert(type(term.gain) == "number", "attitude allocator term gain must be number")

        matrix[row][col] = matrix[row][col] + term.gain * attitude
    end

    return matrix
end

local function identityMatrix()
    return {
        { 1.0, 0.0, 0.0 },
        { 0.0, 1.0, 0.0 },
        { 0.0, 0.0, 1.0 },
    }
end

function attitude_allocator.apply(config, pose, rawCommands)
    assert(type(rawCommands) == "table", "attitude allocator rawCommands must be table")

    local raw = copyCommands(rawCommands)
    local rollDeg = math.deg((pose and pose.roll) or 0.0)
    local pitchDeg = math.deg((pose and pose.pitch) or 0.0)

    if config == nil or config.enabled ~= true then
        return {
            commands = raw,
            debug = {
                enabled = false,
                pitch_deg = pitchDeg,
                roll_deg = rollDeg,
                raw = attitudeCommands(raw),
                fixed = attitudeCommands(raw),
                scheduled = attitudeCommands(raw),
                matrix = identityMatrix(),
            },
        }
    end

    assert(config.model == "affine_tensor", "unsupported attitude allocator model: " .. tostring(config.model))

    local baseMatrix = copyMatrix(config.base_matrix)
    local scheduledMatrix = scheduleMatrix(config, pose)
    local fixed = multiplyMatrixCommands(baseMatrix, raw)
    local scheduled = multiplyMatrixCommands(scheduledMatrix, raw)

    return {
        commands = {
            collective = raw.collective,
            roll = scheduled.roll,
            pitch = scheduled.pitch,
            yaw = scheduled.yaw,
        },
        debug = {
            enabled = true,
            pitch_deg = pitchDeg,
            roll_deg = rollDeg,
            raw = attitudeCommands(raw),
            fixed = fixed,
            scheduled = scheduled,
            matrix = scheduledMatrix,
        },
    }
end

return attitude_allocator
