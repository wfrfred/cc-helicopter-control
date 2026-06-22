local feedforward = {}

function feedforward.linear(gain, bias)
    gain = gain or 0.0
    bias = bias or 0.0

    return function(input)
        return bias + gain * input.target
    end
end

function feedforward.directionalLinear(gain_neg, gain_pos, bias_neg, bias_pos)
    return function(input)
        if input.target < 0.0 then
            return feedforward.linear(gain_neg, bias_neg)(input)
        end

        if input.target > 0.0 then
            return feedforward.linear(gain_pos, bias_pos)(input)
        end

        return 0.0
    end
end

return feedforward
