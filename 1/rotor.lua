local protocol = require("lib.protocol")

local rotor = {}

local Mixer = {}
Mixer.__index = Mixer

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

local function makeTuple(bladeMount, rotorPhase, phaseOffset, collective, roll, pitch)
    local out = {}

    for blade, mount in pairs(bladeMount) do
        local phase = rotorPhase - mount + phaseOffset
        out[blade] = collective + roll * math.sin(phase) + pitch * math.cos(phase)
    end

    return out
end

function rotor.new(config)
    local upper = peripheral.wrap(config.upper_bearing)
    local lower = peripheral.wrap(config.lower_bearing)

    assert(upper, "upper swivel bearing not found: " .. config.upper_bearing)
    assert(lower, "lower swivel bearing not found: " .. config.lower_bearing)

    rednet.open(config.modem_side)

    return setmetatable({
        upper = upper,
        lower = lower,
        phase_offset_upper = config.phase_offset_upper,
        phase_offset_lower = config.phase_offset_lower,
        roll_sign = config.roll_sign,
        pitch_sign = config.pitch_sign,
        yaw_sign = config.yaw_sign,
        blade_mount = config.blade_mount,
        collective_cmd = 0.0,
        roll_cmd = 0.0,
        yaw_cmd = 0.0,
        pitch_cmd = 0.0,
    }, Mixer)
end

function Mixer:set(collective, roll, yaw, pitch)
    self.collective_cmd = tonumber(collective) or 0.0
    self.roll_cmd = tonumber(roll) or 0.0
    self.yaw_cmd = tonumber(yaw) or 0.0
    self.pitch_cmd = tonumber(pitch) or 0.0
end

function Mixer:update()
    local upperPhase = getPhaseRad(self.upper)
    local lowerPhase = getPhaseRad(self.lower)

    local upperCollective = self.collective_cmd + self.yaw_sign * self.yaw_cmd
    local lowerCollective = self.collective_cmd - self.yaw_sign * self.yaw_cmd

    local upperMsg = makeTuple(
        self.blade_mount,
        upperPhase,
        self.phase_offset_upper,
        upperCollective,
        self.roll_sign * self.roll_cmd,
        self.pitch_sign * self.pitch_cmd
    )

    local lowerMsg = makeTuple(
        self.blade_mount,
        lowerPhase,
        self.phase_offset_lower,
        lowerCollective,
        self.roll_sign * self.roll_cmd,
        self.pitch_sign * self.pitch_cmd
    )

    rednet.broadcast(upperMsg, protocol.LAYER.UPPER)
    rednet.broadcast(lowerMsg, protocol.LAYER.LOWER)

    return {
        upperPhase = upperPhase,
        lowerPhase = lowerPhase,
        upper = upperMsg,
        lower = lowerMsg,
    }
end

return rotor
