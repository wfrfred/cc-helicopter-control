local mathx = require("lib.mathx")

local yaw_lock = {}

local Lock = {}
Lock.__index = Lock

function yaw_lock.new(initial_yaw, control)
    return setmetatable({
        control = control,
        target_yaw = initial_yaw,
        was_manual = false,
        pending = false,
    }, Lock)
end

function Lock:update(input_yaw, current_yaw, yaw_rate)
    local manual = input_yaw ~= 0

    if manual then
        self.target_yaw = current_yaw
        self.was_manual = true
        self.pending = false
        return {
            target_yaw = current_yaw,
            yaw_err = 0.0,
            commanded_rate = input_yaw * self.control.yaw_target_rate,
            angle_active = false,
        }
    end

    if self.was_manual then
        self.pending = true
        self.was_manual = false
    end

    if self.pending then
        if math.abs(yaw_rate) < self.control.yaw_lock_rate_deadband then
            self.target_yaw = current_yaw
            self.pending = false
        end
        return {
            target_yaw = self.target_yaw,
            yaw_err = 0.0,
            commanded_rate = 0.0,
            angle_active = false,
        }
    end

    local yaw_err = mathx.wrapPi(self.target_yaw - current_yaw)
    return {
        target_yaw = self.target_yaw,
        yaw_err = yaw_err,
        commanded_rate = 0.0,
        angle_active = true,
    }
end

return yaw_lock
