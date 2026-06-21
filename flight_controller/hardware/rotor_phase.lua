local rotor_phase = {}

local Reader = {}
Reader.__index = Reader

local function getPhaseRad(bearing)
    if bearing.getTargetAngleRad then
        return bearing.getTargetAngleRad()
    end

    if bearing.getAngleRad then
        return bearing.getAngleRad()
    end

    if bearing.getTargetAngle then
        return math.rad(bearing.getTargetAngle())
    end

    if bearing.getAngle then
        return math.rad(bearing.getAngle())
    end

    error("no usable angle getter")
end

function rotor_phase.new(hardware)
    local upper = peripheral.wrap(hardware.upper_bearing)
    local lower = peripheral.wrap(hardware.lower_bearing)

    assert(upper, "upper swivel bearing not found: " .. hardware.upper_bearing)
    assert(lower, "lower swivel bearing not found: " .. hardware.lower_bearing)

    return setmetatable({
        upper = upper,
        lower = lower,
    }, Reader)
end

function Reader:read()
    return {
        upper = getPhaseRad(self.upper),
        lower = getPhaseRad(self.lower),
    }
end

return rotor_phase
