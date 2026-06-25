local mathx = require("lib.mathx")
local tablex = require("lib.tablex")

local allocation = {}

local Allocation = {}
Allocation.__index = Allocation

local AXIS_INDEX = {
    roll = 1,
    pitch = 2,
    yaw = 3,
}

local COMMAND_KEYS = {
    "collective",
    "roll",
    "pitch",
    "yaw",
}

local function commandTerms(commands)
    return tablex.pick(commands, COMMAND_KEYS)
end

local function finalClampCommands(commands, limits)
    return {
        collective = commands.collective,
        roll = mathx.clamp(commands.roll, limits.roll_min, limits.roll_max),
        pitch = mathx.clamp(commands.pitch, limits.pitch_min, limits.pitch_max),
        yaw = mathx.clamp(commands.yaw, limits.yaw_min, limits.yaw_max),
    }
end

local function axisIndex(axis)
    local index = AXIS_INDEX[axis]

    assert(index ~= nil, "unknown allocation axis: " .. tostring(axis))

    return index
end

local function attitudeSignal(config, pose, name)
    local limitDeg = config.attitude_limit_deg or {}

    if name == "roll" then
        return mathx.clamp(pose.roll / math.rad(limitDeg.roll or 30.0), -1.0, 1.0)
    end

    if name == "pitch" then
        return mathx.clamp(pose.pitch / math.rad(limitDeg.pitch or 25.0), -1.0, 1.0)
    end

    error("unknown allocation attitude: " .. tostring(name))
end

local function scheduledRows(config, pose)
    local base = config.base_matrix
    local rows = {
        { base[1][1], base[1][2], base[1][3] },
        { base[2][1], base[2][2], base[2][3] },
        { base[3][1], base[3][2], base[3][3] },
    }

    for _, term in ipairs(config.terms or {}) do
        local row = axisIndex(term.out)
        local col = axisIndex(term.input)
        local attitude = attitudeSignal(config, pose, term.attitude)

        rows[row][col] = rows[row][col] + term.gain * attitude
    end

    return rows
end

local function transformAttitudeCommands(rows, commands)
    return {
        roll = rows[1][1] * commands.roll
            + rows[1][2] * commands.pitch
            + rows[1][3] * commands.yaw,
        pitch = rows[2][1] * commands.roll
            + rows[2][2] * commands.pitch
            + rows[2][3] * commands.yaw,
        yaw = rows[3][1] * commands.roll
            + rows[3][2] * commands.pitch
            + rows[3][3] * commands.yaw,
    }
end

local function allocatedCommands(config, pose, rawCommands)
    if config.enabled ~= true then
        return commandTerms(rawCommands)
    end

    assert(config.model == "affine_tensor", "unsupported allocation model: " .. tostring(config.model))

    local transformed = transformAttitudeCommands(scheduledRows(config, pose), rawCommands)

    return {
        collective = rawCommands.collective,
        roll = transformed.roll,
        pitch = transformed.pitch,
        yaw = transformed.yaw,
    }
end

function allocation.new(control)
    return setmetatable({
        attitudeTransform = control.attitude_allocator,
        outputLimits = control.output_limits,
        lastTerms = {},
    }, Allocation)
end

function Allocation:update(input)
    local allocated = allocatedCommands(
        self.attitudeTransform,
        input.pose,
        input.rawCommands
    )
    local commands = finalClampCommands(allocated, self.outputLimits)

    self.lastTerms = {
        rawCommands = commandTerms(input.rawCommands),
        allocatedCommands = commandTerms(allocated),
        finalCommands = commands,
    }

    return commands
end

function Allocation:terms()
    return self.lastTerms
end

return allocation
