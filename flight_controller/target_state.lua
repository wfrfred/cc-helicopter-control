local mathx = require("lib.mathx")

local target_state = {}

local State = {}
State.__index = State

local function moveToward(x, target, rate, dt)
    local d = target - x
    local step = rate * dt

    if math.abs(d) <= step then
        return target
    end

    if d > 0 then
        return x + step
    end

    return x - step
end

function target_state.new(initial, control)
    return setmetatable({
        control = control,
        height = initial.pos.y,
        roll = control.home_roll,
        pitch = control.home_pitch,
    }, State)
end

function State:update(input, dt)
    local control = self.control

    if input.roll ~= 0 then
        self.roll = mathx.clamp(
            self.roll + input.roll * control.roll_target_rate * dt,
            -control.max_target_roll,
            control.max_target_roll
        )
    else
        self.roll = moveToward(self.roll, control.home_roll, control.roll_center_rate, dt)
    end

    if input.pitch ~= 0 then
        self.pitch = mathx.clamp(
            self.pitch + input.pitch * control.pitch_target_rate * dt,
            -control.max_target_pitch,
            control.max_target_pitch
        )
    else
        self.pitch = moveToward(self.pitch, control.home_pitch, control.pitch_center_rate, dt)
    end

    self.height = self.height + input.climb * control.height_target_rate * dt
end

return target_state
