local config = require("config")
local sync = config.sync

if sync.enabled == true then
    print("startup sync: " .. table.concat(sync.sources, " -> "))

    local ok, err = pcall(function()
        shell.run("sync", table.unpack(sync.sources))
    end)

    if not ok then
        print("startup sync failed: " .. tostring(err))
    end
else
    print("startup sync: disabled")
end

sleep(1)
term.clear()
term.setCursorPos(1, 1)
print("startup: actuator controller")

local protocol = require("lib.protocol")
local pwm = require("pwm")

local MODEM_SIDE = config.modem.side

local LISTEN = config.actuator.listen
local DISPLAY_DT = config.actuator.display_dt

local OUTPUTS = config.actuator.outputs

local lastSender = nil
local lastBadMsg = nil

for _, out in ipairs(OUTPUTS) do
    pwm.set(out.side, 0)
end

local function polarizedValue(value, polarity)
    value = tonumber(value) or 0

    if polarity < 0 then
        value = -value
    end

    return protocol.clamp(value, 0, 15)
end

local function receiverTask()
    rednet.open(MODEM_SIDE)

    while true do
        local sender, msg = rednet.receive(LISTEN)
        lastSender = sender

        if type(msg) == "table" then
            lastBadMsg = nil

            for _, out in ipairs(OUTPUTS) do
                local raw = tonumber(msg[out.blade]) or 0
                local value = polarizedValue(raw, out.polarity)

                pwm.set(out.side, value)
            end
        else
            lastBadMsg = type(msg)
        end
    end
end

local function displayTask()
    while true do
        term.clear()
        term.setCursorPos(1, 1)

        print("actuator pwm")
        print("listen:", LISTEN)
        print("sender:", lastSender)
        print()

        for _, out in ipairs(OUTPUTS) do
            local value = pwm.get(out.side)
            print(("blade=%d polarity=%+d side=%s"):format(out.blade, out.polarity, out.side))
            print(("pwm=%.2f"):format(value))
            print()
        end

        if lastBadMsg then
            print("bad msg:", lastBadMsg)
        end

        sleep(DISPLAY_DT)
    end
end

parallel.waitForAny(receiverTask, pwm.run, displayTask)
