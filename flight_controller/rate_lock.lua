local rate_lock = {}

local Lock = {}
Lock.__index = Lock

local function defaultError(target, current)
    return target - current
end

function rate_lock.new(options)
    return setmetatable({
        target = options.initial_target,
        targetRate = options.target_rate,
        rateDeadband = options.rate_deadband,
        relockTimeout = options.relock_timeout or 0.0,
        error = options.error or defaultError,
        wasManual = false,
        pending = false,
        pendingTime = 0.0,
    }, Lock)
end

function Lock:capture(currentValue)
    self.target = currentValue
    self.wasManual = false
    self.pending = false
    self.pendingTime = 0.0
end

function Lock:update(manualInput, currentValue, measuredRate, dt)
    dt = dt or 0.0

    if manualInput ~= 0 then
        self.target = currentValue
        self.wasManual = true
        self.pending = false
        self.pendingTime = 0.0
        return {
            target = currentValue,
            error = 0.0,
            commandedRate = manualInput * self.targetRate,
            active = false,
            pending = false,
            state = "manual",
        }
    end

    if self.wasManual then
        self.pending = true
        self.pendingTime = 0.0
        self.wasManual = false
    end

    if self.pending then
        self.pendingTime = self.pendingTime + dt

        local stopped = math.abs(measuredRate) < self.rateDeadband
        local timedOut = self.relockTimeout > 0.0 and self.pendingTime >= self.relockTimeout

        if stopped or timedOut then
            self.target = currentValue
            self.pending = false
            self.pendingTime = 0.0
        else
            return {
                target = self.target,
                error = 0.0,
                commandedRate = 0.0,
                active = false,
                pending = true,
                state = "pending",
            }
        end
    end

    return {
        target = self.target,
        error = self.error(self.target, currentValue),
        commandedRate = 0.0,
        active = true,
        pending = false,
        state = "locked",
    }
end

return rate_lock
