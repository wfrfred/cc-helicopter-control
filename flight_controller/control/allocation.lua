local attitude_allocator = require("lib.attitude_allocator")
local mathx = require("lib.mathx")

local allocation = {}

local Allocation = {}
Allocation.__index = Allocation

local function finalClampCommands(commands, limits)
    return {
        collective = commands.collective,
        roll = mathx.clamp(commands.roll, limits.roll_min, limits.roll_max),
        pitch = mathx.clamp(commands.pitch, limits.pitch_min, limits.pitch_max),
        yaw = mathx.clamp(commands.yaw, limits.yaw_min, limits.yaw_max),
    }
end

function allocation.new(control)
    return setmetatable({
        allocator = control.attitude_allocator,
        outputLimits = control.output_limits,
        lastTerms = {},
    }, Allocation)
end

function Allocation:update(input)
    local allocated = attitude_allocator.apply(
        self.allocator,
        input.pose,
        input.rawCommands
    )
    local commands = finalClampCommands(allocated.commands, self.outputLimits)

    self.lastTerms = {
        rawCommands = input.rawCommands,
        allocatedCommands = allocated.commands,
        finalCommands = commands,
        debug = allocated.debug,
    }

    return commands
end

function Allocation:terms()
    return self.lastTerms
end

return allocation
