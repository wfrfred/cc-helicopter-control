local input = {}
local config = require("config")

local TYPEWRITER_NAME = config.input.typewriter_name

local KEY_SPACE = 32
local KEY_A = 65
local KEY_D = 68
local KEY_E = 69
local KEY_Q = 81
local KEY_S = 83
local KEY_W = 87
local KEY_CAPS_LOCK = 280
local KEY_LEFT_SHIFT = 340
local KEY_RIGHT_SHIFT = 344

local tw = peripheral.find(TYPEWRITER_NAME)
assert(tw, TYPEWRITER_NAME .. " not found")

local previousCapsLock = false

local function boolToAxis(pos, neg)
    if pos and not neg then
        return 1
    end

    if neg and not pos then
        return -1
    end

    return 0
end

local function getPressedSet()
    local codes = tw.getPressedKeyCodes()
    local set = {}

    if type(codes) ~= "table" then
        return set
    end

    for _, code in pairs(codes) do
        code = tonumber(code)
        if code then
            set[code] = true
        end
    end

    return set
end

function input.read()
    local key = getPressedSet()

    local w = key[KEY_W]
    local s = key[KEY_S]
    local a = key[KEY_A]
    local d = key[KEY_D]
    local q = key[KEY_Q]
    local e = key[KEY_E]
    local space = key[KEY_SPACE]
    local shift = key[KEY_LEFT_SHIFT] or key[KEY_RIGHT_SHIFT]
    local capsLock = key[KEY_CAPS_LOCK] == true
    local cruiseLock = capsLock and not previousCapsLock

    previousCapsLock = capsLock

    return {
        controls = {
            roll = boolToAxis(e, q),
            pitch = boolToAxis(w, s),
            heading = boolToAxis(d, a),
            climb = boolToAxis(space, shift),
        },
        event = {
            cruiseLock = cruiseLock,
        },
    }
end

return input
