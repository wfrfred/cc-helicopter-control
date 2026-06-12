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

function rotor.new(hardware, calibration, mixerAxis)
    local upper = peripheral.wrap(hardware.upper_bearing)
    local lower = peripheral.wrap(hardware.lower_bearing)

    assert(upper, "upper swivel bearing not found: " .. hardware.upper_bearing)
    assert(lower, "lower swivel bearing not found: " .. hardware.lower_bearing)

    rednet.open(hardware.modem_side)

    return setmetatable({
        upper = upper,
        lower = lower,
        phase_offset_upper = calibration.phase_offset_upper,
        phase_offset_lower = calibration.phase_offset_lower,
        mixer_axis = mixerAxis,
        blade_mount = hardware.blade_mount,
        collective_cmd = 0.0,
        roll_cmd = 0.0,
        yaw_cmd = 0.0,
        pitch_cmd = 0.0,
    }, Mixer)
end

function Mixer:set(collective, roll, yaw, pitch)
    self.collective_cmd = collective
    self.roll_cmd = roll
    self.yaw_cmd = yaw
    self.pitch_cmd = pitch
end

function Mixer:update()
    local upperPhase = getPhaseRad(self.upper)
    local lowerPhase = getPhaseRad(self.lower)
    local collectiveCmd = self.mixer_axis.collective * self.collective_cmd
    local rollCmd = self.mixer_axis.roll * self.roll_cmd
    local pitchCmd = self.mixer_axis.pitch * self.pitch_cmd
    local yawCmd = self.mixer_axis.yaw * self.yaw_cmd

    local upperCollective = collectiveCmd + yawCmd
    local lowerCollective = collectiveCmd - yawCmd

    local upperMsg = makeTuple(
        self.blade_mount,
        upperPhase,
        self.phase_offset_upper,
        upperCollective,
        rollCmd,
        pitchCmd
    )

    local lowerMsg = makeTuple(
        self.blade_mount,
        lowerPhase,
        self.phase_offset_lower,
        lowerCollective,
        rollCmd,
        pitchCmd
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
