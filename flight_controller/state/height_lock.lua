local height_lock = {}

local Lock = {}
Lock.__index = Lock

function height_lock.new(options)
    return setmetatable({
        target = options.initial_target,
        targetRate = options.target_rate,
        rateDeadband = options.rate_deadband,
        relockTimeout = options.relock_timeout or 0.0,
        wasManual = false,
        pending = false,
        pendingTime = 0.0,
    }, Lock)
end

function Lock:capture(height)
    self.target = height
    self.wasManual = false
    self.pending = false
    self.pendingTime = 0.0
end

function Lock:lockedTarget(height)
    self:capture(height)

    return {
        target = self.target,
        speed = 0.0,
        active = true,
        pending = false,
        error = 0.0,
        source = "locked",
    }
end

function Lock:update(input)
    local climb = input.climb or 0.0
    local height = input.height
    local verticalSpeed = input.verticalSpeed or 0.0
    local dt = input.dt or 0.0

    if climb ~= 0.0 then
        self.target = height
        self.wasManual = true
        self.pending = false
        self.pendingTime = 0.0

        return {
            target = height,
            speed = climb * self.targetRate,
            active = false,
            pending = false,
            error = 0.0,
            source = "manual",
        }
    end

    if self.wasManual then
        self.pending = true
        self.pendingTime = 0.0
        self.wasManual = false
    end

    if self.pending then
        self.pendingTime = self.pendingTime + dt

        local stopped = math.abs(verticalSpeed) < self.rateDeadband
        local timedOut = self.relockTimeout > 0.0 and self.pendingTime >= self.relockTimeout

        if stopped or timedOut then
            self:capture(height)
        else
            return {
                target = self.target,
                speed = 0.0,
                active = false,
                pending = true,
                error = 0.0,
                source = "pending",
            }
        end
    end

    return {
        target = self.target,
        speed = 0.0,
        active = true,
        pending = false,
        error = self.target - height,
        source = "locked",
    }
end

return height_lock
