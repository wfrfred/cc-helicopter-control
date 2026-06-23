local lock = {}

local Lock = {}
Lock.__index = Lock

local function identity(value)
    return value
end

local function linearError(target, value)
    return target - value
end

function lock.new(options)
    local normalize = options.normalize or identity

    return setmetatable({
        target = normalize(options.initial),
        targetRate = options.target_rate,
        rateDeadband = options.rate_deadband,
        relockTimeout = options.relock_timeout or 0.0,
        normalize = normalize,
        error = options.error or linearError,
        wasManual = false,
        pending = false,
        pendingTime = 0.0,
    }, Lock)
end

function Lock:capture(value)
    self.target = self.normalize(value)
    self.wasManual = false
    self.pending = false
    self.pendingTime = 0.0
end

function Lock:targetFor(value, rate, active, pending, source)
    local target = self.normalize(self.target)
    local current = self.normalize(value)

    return {
        target = target,
        rate = rate or 0.0,
        active = active,
        pending = pending,
        error = self.error(target, current),
        source = source,
    }
end

function Lock:locked(value)
    self:capture(value)

    return self:targetFor(value, 0.0, true, false, "locked")
end

function Lock:update(input)
    local command = input.input or 0.0
    local value = input.value
    local rate = input.rate or 0.0
    local dt = input.dt or 0.0

    if command ~= 0.0 then
        self.target = self.normalize(value)
        self.wasManual = true
        self.pending = false
        self.pendingTime = 0.0

        return self:targetFor(value, command * self.targetRate, false, false, "manual")
    end

    if self.wasManual then
        self.pending = true
        self.pendingTime = 0.0
        self.wasManual = false
    end

    if self.pending then
        self.pendingTime = self.pendingTime + dt

        local stopped = math.abs(rate) < self.rateDeadband
        local timedOut = self.relockTimeout > 0.0 and self.pendingTime >= self.relockTimeout

        if stopped or timedOut then
            self:capture(value)
        else
            return self:targetFor(value, 0.0, false, true, "pending")
        end
    end

    return self:targetFor(value, 0.0, true, false, "locked")
end

return lock
