local common = require("modes.common")

local target = {}

local Target = {}
Target.__index = Target

local builders = {
    manual = require("modes.manual"),
    position_hold = require("modes.position_hold"),
    cruise = require("modes.cruise"),
    navigation = require("modes.navigation"),
}

function target.new()
    return setmetatable({}, Target)
end

function Target:update(request)
    local mode = request.mode
    local builder = builders[mode.name]

    assert(builder ~= nil, "unknown mode target: " .. tostring(mode.name))
    assert(type(builder.target) == "function", "mode target must expose target(input): " .. tostring(mode.name))

    return builder.target({
        mode = mode,
        command = request.input,
        state = request.state,
        vertical = common.verticalFromLock(request.height),
        heading = common.headingFromLock(request.heading),
        dt = request.dt,
    })
end

return target
