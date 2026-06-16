local attitude_decoupler = {}

local function copyCommands(commands)
    return {
        collective = commands.collective,
        roll = commands.roll,
        pitch = commands.pitch,
        yaw = commands.yaw,
    }
end

local function matrixValue(matrix, row, col)
    assert(type(matrix[row]) == "table", "attitude decoupler matrix row must be table")
    assert(type(matrix[row][col]) == "number", "attitude decoupler matrix value must be number")

    return matrix[row][col]
end

function attitude_decoupler.apply(config, commands)
    assert(type(commands) == "table", "attitude decoupler commands must be table")

    if config == nil or config.enabled ~= true then
        return copyCommands(commands)
    end

    local matrix = config.matrix

    assert(type(matrix) == "table", "attitude decoupler matrix must be table")

    local roll = commands.roll
    local pitch = commands.pitch
    local yaw = commands.yaw

    return {
        collective = commands.collective,
        roll = matrixValue(matrix, 1, 1) * roll
            + matrixValue(matrix, 1, 2) * pitch
            + matrixValue(matrix, 1, 3) * yaw,
        pitch = matrixValue(matrix, 2, 1) * roll
            + matrixValue(matrix, 2, 2) * pitch
            + matrixValue(matrix, 2, 3) * yaw,
        yaw = matrixValue(matrix, 3, 1) * roll
            + matrixValue(matrix, 3, 2) * pitch
            + matrixValue(matrix, 3, 3) * yaw,
    }
end

return attitude_decoupler
