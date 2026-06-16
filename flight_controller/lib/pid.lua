local pid = {}

local Controller = {}
Controller.__index = Controller

local function zeroFeedforward()
    return 0.0
end

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
        kp = config.kp or 0.0,
        ki = config.ki or 0.0,
        kd = config.kd or 0.0,

        i_min = config.i_min,
        i_max = config.i_max,

        out_min = config.out_min,
        out_max = config.out_max,

        deadband = config.deadband or 0.0,
        feedforward = config.feedforward or zeroFeedforward,

        integral = 0.0,
        last_error = nil,
        last_output = 0.0,
        last_terms = {
            p = 0.0,
            i = 0.0,
            d = 0.0,
            raw = 0.0,
            ff = 0.0,
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
        ff = 0.0,
        output = 0.0,
    }
end

function Controller:setFeedforward(feedforward)
    self.feedforward = feedforward or zeroFeedforward
end

function Controller:setGains(kp, ki, kd)
    self.kp = kp
    self.ki = ki
    self.kd = kd
end

function Controller:update(input)
    assert(type(input) == "table", "pid update input must be a table")

    local target = input.target
    local current = input.current
    local dt = input.dt

    assert(dt > 0, "pid dt must be positive")
    assert(target ~= nil, "pid target must be set")
    assert(current ~= nil, "pid current must be set")

    local error = input.error
    if error == nil then
        error = target - current
    end

    if math.abs(error) < self.deadband then
        error = 0.0
    end

    self.integral = self.integral + error * dt
    self.integral = clamp(self.integral, self.i_min, self.i_max)

    local derivative = input.derivative or 0.0
    if input.derivative == nil and self.last_error ~= nil then
        derivative = (error - self.last_error) / dt
    end

    local p_term = self.kp * error
    local i_term = self.ki * self.integral
    local d_term = self.kd * derivative
    local raw_output = p_term + i_term + d_term
    local feedforward = self.feedforward({
        target = target,
        current = current,
        error = error,
        derivative = derivative,
        integral = self.integral,
        dt = dt,
    }) or 0.0
    local output = clamp(raw_output + feedforward, self.out_min, self.out_max)

    self.last_error = error
    self.last_output = output
    self.last_terms = {
        p = p_term,
        i = i_term,
        d = d_term,
        raw = raw_output,
        ff = feedforward,
        output = output,
    }

    return {
        target = target,
        current = current,
        error = error,
        integral = self.integral,
        derivative = derivative,
        output = output,
        terms = self.last_terms,
    }
end

function Controller:last()
    return self.last_output
end

function Controller:terms()
    return self.last_terms
end

return pid
