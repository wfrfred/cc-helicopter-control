local height_lock = {}

local Lock = {}
Lock.__index = Lock

function height_lock.new(initial_height, control)
    return setmetatable({
        control = control,
        target_height = initial_height,
        was_manual = false,
        pending = false,
    }, Lock)
end

function Lock:update(input_climb, current_height, vertical_speed)
    local manual = input_climb ~= 0

    if manual then
        self.target_height = current_height
        self.was_manual = true
        self.pending = false
        return {
            target_height = current_height,
            height_err = 0.0,
            commanded_vertical_speed = input_climb * self.control.height_target_rate,
            lock_active = false,
        }
    end

    if self.was_manual then
        self.pending = true
        self.was_manual = false
    end

    if self.pending then
        if math.abs(vertical_speed) < self.control.height_lock_speed_deadband then
            self.target_height = current_height
            self.pending = false
        end
        return {
            target_height = self.target_height,
            height_err = 0.0,
            commanded_vertical_speed = 0.0,
            lock_active = false,
        }
    end

    return {
        target_height = self.target_height,
        height_err = self.target_height - current_height,
        commanded_vertical_speed = 0.0,
        lock_active = true,
    }
end

return height_lock
