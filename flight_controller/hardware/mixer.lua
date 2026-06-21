local mixer = {}

local Mixer = {}
Mixer.__index = Mixer

local function makeTuple(bladeMount, rotorPhase, phaseOffset, collective, roll, pitch)
    local out = {}

    for blade, mount in pairs(bladeMount) do
        local phase = rotorPhase + mount + phaseOffset
        out[blade] = collective + roll * math.sin(phase) + pitch * math.cos(phase)
    end

    return out
end

function mixer.new(hardware, calibration)
    return setmetatable({
        phaseOffsetUpper = calibration.phase_offset_upper,
        phaseOffsetLower = calibration.phase_offset_lower,
        bladeMount = hardware.blade_mount,
    }, Mixer)
end

function Mixer:update(input)
    local commands = input.commands
    local phase = input.phase
    local upperCollective = commands.collective - commands.yaw
    local lowerCollective = commands.collective + commands.yaw
    local upper = makeTuple(
        self.bladeMount,
        phase.upper,
        self.phaseOffsetUpper,
        upperCollective,
        commands.roll,
        commands.pitch
    )
    local lower = makeTuple(
        self.bladeMount,
        phase.lower,
        self.phaseOffsetLower,
        lowerCollective,
        commands.roll,
        commands.pitch
    )

    return {
        phase = {
            upper = phase.upper,
            lower = phase.lower,
        },
        blades = {
            upper = upper,
            lower = lower,
        },
    }
end

return mixer
