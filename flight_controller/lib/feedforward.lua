local feedforward = {}

function feedforward.linear(gain, bias)
    gain = gain or 0.0
    bias = bias or 0.0

    return function(input)
        return bias + gain * input.target
    end
end

return feedforward
