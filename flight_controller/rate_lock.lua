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
        error = options.error or defaultError,
        wasManual = false,
        pending = false,
    }, Lock)
end

function Lock:update(input, current, rate)
    if input ~= 0 then
        self.target = current
        self.wasManual = true
        self.pending = false
        return {
            target = current,
            error = 0.0,
            commandedRate = input * self.targetRate,
            active = false,
        }
    end

    if self.wasManual then
        self.pending = true
        self.wasManual = false
    end

    if self.pending then
        if math.abs(rate) < self.rateDeadband then
            self.target = current
            self.pending = false
        end
        return {
            target = self.target,
            error = 0.0,
            commandedRate = 0.0,
            active = false,
        }
    end

    return {
        target = self.target,
        error = self.error(self.target, current),
        commandedRate = 0.0,
        active = true,
    }
end

return rate_lock
