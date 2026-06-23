local attitude_allocator = {}

local AXIS_INDEX = {
    roll = 1,
    pitch = 2,
    yaw = 3,
}

local AXES = { "roll", "pitch", "yaw" }

local function matrixApi()
    assert(type(matrix) == "table", "matrix API must be loaded")
    assert(type(matrix.from2DArray) == "function", "matrix.from2DArray must be available")

    return matrix
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

local function matrixFromRows(rows)
    local out = {}

    assert(type(rows) == "table", "attitude allocator matrix must be table")

    for row = 1, 3 do
        out[row] = {}

        for col = 1, 3 do
            out[row][col] = matrixValue(rows, row, col)
        end
    end

    return matrixApi().from2DArray(out)
end

local function matrixRows(value)
    return {
        { value[1][1], value[1][2], value[1][3] },
        { value[2][1], value[2][2], value[2][3] },
        { value[3][1], value[3][2], value[3][3] },
    }
end

local function commandsColumn(commands)
    return matrixApi().from2DArray({
        { commands.roll },
        { commands.pitch },
        { commands.yaw },
    })
end

local function transformCommands(transform, commands)
    local output = transform * commandsColumn(commands)

    return {
        roll = output[1][1],
        pitch = output[2][1],
        yaw = output[3][1],
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
    local transform = matrixFromRows(config.base_matrix)

    for _, term in ipairs(config.terms or {}) do
        local row = axisIndex(term.out)
        local col = axisIndex(term.input)
        local attitude = attitudeSignal(config, pose, term.attitude)

        assert(type(term.gain) == "number", "attitude allocator term gain must be number")

        transform[row][col] = transform[row][col] + term.gain * attitude
    end

    return transform
end

local function identityMatrix()
    return {
        { 1.0, 0.0, 0.0 },
        { 0.0, 1.0, 0.0 },
        { 0.0, 0.0, 1.0 },
    }
end

function attitude_allocator.apply(config, pose, rawCommands)
    local rollDeg = math.deg((pose and pose.roll) or 0.0)
    local pitchDeg = math.deg((pose and pose.pitch) or 0.0)

    if config == nil or config.enabled ~= true then
        return {
            commands = rawCommands,
            debug = {
                enabled = false,
                pitch_deg = pitchDeg,
                roll_deg = rollDeg,
                raw = attitudeCommands(rawCommands),
                fixed = attitudeCommands(rawCommands),
                scheduled = attitudeCommands(rawCommands),
                matrix = identityMatrix(),
            },
        }
    end

    assert(config.model == "affine_tensor", "unsupported attitude allocator model: " .. tostring(config.model))

    local baseMatrix = matrixFromRows(config.base_matrix)
    local scheduledMatrix = scheduleMatrix(config, pose)
    local fixed = transformCommands(baseMatrix, rawCommands)
    local scheduled = transformCommands(scheduledMatrix, rawCommands)

    return {
        commands = {
            collective = rawCommands.collective,
            roll = scheduled.roll,
            pitch = scheduled.pitch,
            yaw = scheduled.yaw,
        },
        debug = {
            enabled = true,
            pitch_deg = pitchDeg,
            roll_deg = rollDeg,
            raw = attitudeCommands(rawCommands),
            fixed = fixed,
            scheduled = scheduled,
            matrix = matrixRows(scheduledMatrix),
        },
    }
end

return attitude_allocator
