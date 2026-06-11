local protocol = {}

protocol.CONTROL = {
    INPUT = "control_input",
    TELEMETRY = "control_telemetry",
}

function protocol.clamp(x, lo, hi)
    x = tonumber(x) or 0
    if x < lo then return lo end
    if x > hi then return hi end
    return x
end

return protocol
