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

local function scheduledMatrix(transform, pose)
    if #transform.terms == 0 then
        return transform.baseMatrix
    end

    local out = transform.baseMatrix:clone()

    for _, term in ipairs(transform.terms) do
        local attitude = nil

        if term.axis == "roll" then
            attitude = pose.roll
        elseif term.axis == "pitch" then
            attitude = pose.pitch
        end

        out[term.row][term.col] = out[term.row][term.col]
            + term.gain * mathx.clamp(attitude / term.limit, -1.0, 1.0)
    end

    return out
end

function allocation.new(control)
    local config = control.attitude_allocator
    local enabled = config.enabled == true
    local transform = {
        enabled = enabled,
    }

    if enabled then
        assert(config.model == "affine_tensor", "unsupported allocation model: " .. tostring(config.model))

        local limitDeg = config.attitude_limit_deg or {}

        transform.baseMatrix = matrix.from2DArray(config.base_matrix)
        transform.terms = tablex.list.map(config.terms or {}, function(term)
            local row = AXIS_INDEX[term.out]
            local col = AXIS_INDEX[term.input]
            local limit = nil

            assert(row ~= nil, "unknown allocation axis: " .. tostring(term.out))
            assert(col ~= nil, "unknown allocation axis: " .. tostring(term.input))

            if term.attitude == "roll" then
                limit = math.rad(limitDeg.roll or 30.0)
            elseif term.attitude == "pitch" then
                limit = math.rad(limitDeg.pitch or 25.0)
            else
                error("unknown allocation attitude: " .. tostring(term.attitude))
            end

            return {
                row = row,
                col = col,
                axis = term.attitude,
                gain = term.gain,
                limit = limit,
            }
        end)
    end

    return setmetatable({
        attitudeTransform = transform,
        outputLimits = control.output_limits,
    }, Allocation)
end

function Allocation:reset() end

function Allocation:update(state, target, _feedforwardInput, _dt)
    local rawCommands = target.commands
    local allocated = tablex.record.pick(rawCommands, COMMAND_KEYS)

    if self.attitudeTransform.enabled == true then
        local transformed = scheduledMatrix(self.attitudeTransform, state.pose)
            * matrix.from2DArray({
                { rawCommands.roll },
                { rawCommands.pitch },
                { rawCommands.yaw },
            })

        allocated = {
            collective = rawCommands.collective,
            roll = transformed[1][1],
            pitch = transformed[2][1],
            yaw = transformed[3][1],
        }
    end

    local commands = {
        collective = allocated.collective,
        roll = mathx.clamp(allocated.roll, self.outputLimits.roll_min, self.outputLimits.roll_max),
        pitch = mathx.clamp(allocated.pitch, self.outputLimits.pitch_min, self.outputLimits.pitch_max),
        yaw = mathx.clamp(allocated.yaw, self.outputLimits.yaw_min, self.outputLimits.yaw_max),
    }

    return {
        output = commands,
        terms = {
            rawCommands = tablex.record.pick(rawCommands, COMMAND_KEYS),
            allocatedCommands = tablex.record.pick(allocated, COMMAND_KEYS),
            finalCommands = commands,
        },
    }
end

return allocation
