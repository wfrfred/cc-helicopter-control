local mathx = require("lib.mathx")

local heading_lock = {}

local Lock = {}
Lock.__index = Lock

local function makeTarget(angle, heading, rate, active, pending, source)
    local wrapped = mathx.wrapPi(angle)

    return {
        angle = wrapped,
        rate = rate or 0.0,
        active = active,
        pending = pending,
        error = mathx.wrapPi(wrapped - heading),
        source = source,
    }
end

function heading_lock.new(options)
    return setmetatable({
        target = mathx.wrapPi(options.initial_heading),
        lookaheadRate = options.lookahead_rate,
        lookaheadTimeConstant = options.lookahead_time_constant,
        rateDeadband = options.rate_deadband,
        relockTimeout = options.relock_timeout or 0.0,
        wasManual = false,
        pending = false,
        pendingTime = 0.0,
    }, Lock)
end

function Lock:capture(heading)
    self.target = mathx.wrapPi(heading)
    self.wasManual = false
    self.pending = false
    self.pendingTime = 0.0
end

function Lock:lockedTarget(heading)
    self:capture(heading)

    return makeTarget(self.target, heading, 0.0, true, false, "locked")
end

function Lock:update(input)
    local headingInput = input.headingInput or 0.0
    local heading = input.heading
    local headingRate = input.headingRate or 0.0
    local dt = input.dt or 0.0

    if headingInput ~= 0.0 then
        self.target = mathx.wrapPi(heading)
        self.wasManual = true
        self.pending = false
        self.pendingTime = 0.0

        return makeTarget(
            heading + headingInput * self.lookaheadRate * self.lookaheadTimeConstant,
            heading,
            headingInput,
            true,
            false,
            "manual_lookahead"
        )
    end

    if self.wasManual then
        self.pending = true
        self.pendingTime = 0.0
        self.wasManual = false
    end

    if self.pending then
        self.pendingTime = self.pendingTime + dt

        local stopped = math.abs(headingRate) < self.rateDeadband
        local timedOut = self.relockTimeout > 0.0 and self.pendingTime >= self.relockTimeout

        if stopped or timedOut then
            self:capture(heading)
        else
            return makeTarget(heading, heading, 0.0, false, true, "pending")
        end
    end

    return makeTarget(self.target, heading, 0.0, true, false, "locked")
end

return heading_lock
