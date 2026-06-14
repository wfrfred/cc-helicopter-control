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
        roll = control.attitude.home.roll,
        pitch = control.attitude.home.pitch,
    }, State)
end

function State:update(input, dt)
    local control = self.control

    if input.roll ~= 0 then
        self.roll = mathx.clamp(
            self.roll + input.roll * control.attitude.target_rate.roll * dt,
            -control.attitude.limit.roll,
            control.attitude.limit.roll
        )
    else
        self.roll = moveToward(self.roll, control.attitude.home.roll, control.attitude.center_rate.roll, dt)
    end

    if input.pitch ~= 0 then
        self.pitch = mathx.clamp(
            self.pitch + input.pitch * control.attitude.target_rate.pitch * dt,
            -control.attitude.limit.pitch,
            control.attitude.limit.pitch
        )
    else
        self.pitch = moveToward(self.pitch, control.attitude.home.pitch, control.attitude.center_rate.pitch, dt)
    end

end

return target_state
