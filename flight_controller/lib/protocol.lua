local protocol = {}

protocol.LAYER = {
    UPPER = "upper",
    LOWER = "lower",
}

protocol.CONTROL = {
    INPUT = "control_input",
    TELEMETRY = "control_telemetry",
}

protocol.BLADE = {
    FRONT = 1,
    RIGHT = 2,
    BACK = 3,
    LEFT = 4,
}

protocol.SIDE = {
    FRONT = "front",
    BACK = "back",
    LEFT = "left",
    RIGHT = "right",
    TOP = "top",
    BOTTOM = "bottom",
}

protocol.SIGN = {
    POS = 1,
    NEG = -1,
}

function protocol.clamp(x, lo, hi)
    if x < lo then return lo end
    if x > hi then return hi end
    return x
end

function protocol.analog(x)
    return math.floor(protocol.clamp(x, 0, 15) + 0.5)
end

function protocol.signedOutput(value, sign)
    if sign == protocol.SIGN.POS then
        return protocol.analog(value)
    end

    return protocol.analog(-value)
end

return protocol
