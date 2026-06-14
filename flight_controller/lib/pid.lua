local pid = {}

local Controller = {}
Controller.__index = Controller

local function clamp(x, lo, hi)
    if lo and x < lo then
        return lo
    end
    if hi and x > hi then
        return hi
    end
    return x
end

function pid.new(config)
    local self = {
        kp = config.kp,
        ki = config.ki,
        kd = config.kd,

        i_min = config.i_min,
        i_max = config.i_max,

        out_min = config.out_min,
        out_max = config.out_max,

        deadband = config.deadband,

        integral = 0.0,
        last_error = nil,
        last_output = 0.0,
        last_terms = {
            p = 0.0,
            i = 0.0,
            d = 0.0,
            raw = 0.0,
            output = 0.0,
        },
    }

    return setmetatable(self, Controller)
end

function Controller:reset()
    self.integral = 0.0
    self.last_error = nil
    self.last_output = 0.0
    self.last_terms = {
        p = 0.0,
        i = 0.0,
        d = 0.0,
        raw = 0.0,
        output = 0.0,
    }
end

function Controller:setGains(kp, ki, kd)
    self.kp = kp
    self.ki = ki
    self.kd = kd
end

function Controller:update(target, feedback, dt, externalDerivative)
    assert(dt > 0, "pid dt must be positive")

    local error = target - feedback

    if math.abs(error) < self.deadband then
        error = 0.0
    end

    self.integral = self.integral + error * dt
    self.integral = clamp(self.integral, self.i_min, self.i_max)

    local derivative = externalDerivative or 0.0
    if externalDerivative == nil and self.last_error ~= nil then
        derivative = (error - self.last_error) / dt
    end

    local p_term = self.kp * error
    local i_term = self.ki * self.integral
    local d_term = self.kd * derivative
    local raw_output = p_term + i_term + d_term
    local output = raw_output

    output = clamp(output, self.out_min, self.out_max)

    self.last_error = error
    self.last_output = output
    self.last_terms = {
        p = p_term,
        i = i_term,
        d = d_term,
        raw = raw_output,
        output = output,
    }

    return output, error, self.integral, derivative, self.last_terms
end

function Controller:last()
    return self.last_output
end

function Controller:terms()
    return self.last_terms
end

return pid
