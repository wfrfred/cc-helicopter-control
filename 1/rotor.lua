local protocol = require("lib.protocol")
local config = require("config")

local rotor = {}

local MODEM_SIDE = config.rotor.modem_side

local UPPER_BEARING = config.rotor.upper_bearing
local LOWER_BEARING = config.rotor.lower_bearing

local PHASE_OFFSET_UPPER = config.rotor.phase_offset_upper
local PHASE_OFFSET_LOWER = config.rotor.phase_offset_lower

local ROLL_SIGN = config.rotor.roll_sign
local PITCH_SIGN = config.rotor.pitch_sign
local YAW_SIGN = config.rotor.yaw_sign

local upper = peripheral.wrap(UPPER_BEARING)
local lower = peripheral.wrap(LOWER_BEARING)

assert(upper, "upper swivel bearing not found: " .. UPPER_BEARING)
assert(lower, "lower swivel bearing not found: " .. LOWER_BEARING)

rednet.open(MODEM_SIDE)

local collective_cmd = 0.0
local roll_cmd = 0.0
local yaw_cmd = 0.0
local pitch_cmd = 0.0

local BLADE_MOUNT = config.rotor.blade_mount

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

local function makeTuple(rotorPhase, phaseOffset, collective, roll, pitch)
    local out = {}

    for blade, mount in pairs(BLADE_MOUNT) do
        local phase = rotorPhase - mount + phaseOffset
        out[blade] = collective + roll * math.sin(phase) + pitch * math.cos(phase)
    end

    return out
end

function rotor.set(collective, roll, yaw, pitch)
    collective_cmd = tonumber(collective) or 0.0
    roll_cmd = tonumber(roll) or 0.0
    yaw_cmd = tonumber(yaw) or 0.0
    pitch_cmd = tonumber(pitch) or 0.0
end

function rotor.update()
    local upperPhase = getPhaseRad(upper)
    local lowerPhase = getPhaseRad(lower)

    local upperCollective = collective_cmd + YAW_SIGN * yaw_cmd
    local lowerCollective = collective_cmd - YAW_SIGN * yaw_cmd

    local upperMsg = makeTuple(
        upperPhase,
        PHASE_OFFSET_UPPER,
        upperCollective,
        ROLL_SIGN * roll_cmd,
        PITCH_SIGN * pitch_cmd
    )

    local lowerMsg = makeTuple(
        lowerPhase,
        PHASE_OFFSET_LOWER,
        lowerCollective,
        ROLL_SIGN * roll_cmd,
        PITCH_SIGN * pitch_cmd
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
