local mathx = require("lib.mathx")

local pid = {}

local Controller = {}
Controller.__index = Controller

function pid.new(config)
    local self = {
        kp = config.kp or 0.0,
        ki = config.ki or 0.0,
        kd = config.kd or 0.0,

        i_min = config.i_min,
        i_max = config.i_max,

        out_min = config.out_min,
        out_max = config.out_max,

        deadband = config.deadband or 0.0,

        integral = 0.0,
        last_error = nil,
        last_current = nil,
    }

    return setmetatable(self, Controller)
end

function Controller:reset()
    self.integral = 0.0
    self.last_error = nil
    self.last_current = nil
end

function Controller:update(target, current, dt, derivative)
    assert(dt > 0, "pid dt must be positive")
    assert(target ~= nil, "pid target must be set")
    assert(current ~= nil, "pid current must be set")

    local error = target - current

    if math.abs(error) < self.deadband then
        error = 0.0
    end

    if self.last_error ~= nil then
        self.integral = self.integral + (self.last_error + error) * 0.5 * dt
    else
        self.integral = self.integral + error * dt
    end

    self.integral = mathx.clamp(self.integral, self.i_min, self.i_max)

    local currentDerivative = derivative
    if currentDerivative == nil then
        currentDerivative = 0.0

        if self.last_current ~= nil then
            currentDerivative = (current - self.last_current) / dt
        end
    end

    local p_term = self.kp * error
    local i_term = self.ki * self.integral
    local d_term = -self.kd * currentDerivative
    local raw_output = p_term + i_term + d_term
    local output = mathx.clamp(raw_output, self.out_min, self.out_max)
    local terms = {
        error = error,
        derivative = currentDerivative,
        integral = self.integral,
        p = p_term,
        i = i_term,
        d = d_term,
        raw = raw_output,
    }

    self.last_error = error
    self.last_current = current

    return {
        output = output,
        terms = terms,
    }
end

return pid
