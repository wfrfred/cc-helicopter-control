local args = { ... }

local function usage()
    print("Usage: lua tools/generate_actuator_config.lua <old-startup.lua> [config.lua] [--source actuator_controller] [--display-dt 0.5] [--sync-enabled true|false]")
end

local inputPath = nil
local outputPath = "config.lua"
local outputSet = false
local source = "actuator_controller"
local displayDt = nil
local syncEnabled = true

local i = 1
while i <= #args do
    local arg = args[i]

    if arg == "--source" then
        source = args[i + 1]
        i = i + 2
    elseif arg == "--display-dt" then
        displayDt = tonumber(args[i + 1])
        i = i + 2
    elseif arg == "--sync-enabled" then
        local value = args[i + 1]
        syncEnabled = value == "true" or value == "1" or value == "yes"
        i = i + 2
    elseif arg == "--help" or arg == "-h" then
        usage()
        return
    elseif not inputPath then
        inputPath = arg
        i = i + 1
    elseif not outputSet then
        outputPath = arg
        outputSet = true
        i = i + 1
    else
        usage()
        error("unexpected argument: " .. tostring(arg), 0)
    end
end

if not inputPath or not source then
    usage()
    error("missing required input", 0)
end

local function readFile(path)
    if fs and fs.open then
        local file = fs.open(path, "r")
        if not file then
            error("cannot open " .. path, 0)
        end

        local data = file.readAll() or ""
        file.close()
        return data
    end

    local file, err = io.open(path, "rb")
    if not file then
        error(("cannot open %s: %s"):format(path, tostring(err)), 0)
    end

    local data = file:read("*a") or ""
    file:close()
    return data
end

local function writeFile(path, data)
    if fs and fs.open then
        local file = fs.open(path, "w")
        if not file then
            error("cannot open " .. path .. " for writing", 0)
        end

        file.write(data)
        file.close()
        return
    end

    local file, err = io.open(path, "wb")
    if not file then
        error(("cannot open %s for writing: %s"):format(path, tostring(err)), 0)
    end

    file:write(data)
    file:close()
end

local function loadWithEnv(source, name, env)
    if loadstring then
        local chunk, err = loadstring(source, name)
        if not chunk then
            error(err, 0)
        end

        setfenv(chunk, env)
        return chunk
    end

    local chunk, err = load(source, name, "t", env)
    if not chunk then
        error(err, 0)
    end

    return chunk
end

local function protocolStub()
    return {
        LAYER = {
            UPPER = "upper",
            LOWER = "lower",
        },
        BLADE = {
            FRONT = 1,
            RIGHT = 2,
            BACK = 3,
            LEFT = 4,
        },
        SIDE = {
            FRONT = "front",
            BACK = "back",
            LEFT = "left",
            RIGHT = "right",
            TOP = "top",
            BOTTOM = "bottom",
        },
        SIGN = {
            POS = 1,
            NEG = -1,
        },
        clamp = function(value, lo, hi)
            value = tonumber(value) or 0
            if value < lo then return lo end
            if value > hi then return hi end
            return value
        end,
    }
end

local function extractDisplayDt(source)
    local explicit = source:match("local%s+DISPLAY_DT%s*=%s*([%d%.]+)")
    if explicit then
        return tonumber(explicit)
    end

    local bodyStart = source:find("local%s+function%s+displayTask%s*%(")
    local bodyEnd = bodyStart and source:find("parallel%.waitForAny", bodyStart)
    local body = bodyStart and bodyEnd and source:sub(bodyStart, bodyEnd - 1) or source
    local last = nil

    for value in body:gmatch("sleep%s*%(%s*([%d%.]+)%s*%)") do
        last = tonumber(value)
    end

    return last
end

local function readOldStartup(path)
    local source = readFile(path)
    local replacement = [[
return {
    modem_side = MODEM_SIDE,
    listen = LISTEN,
    outputs = OUTPUTS,
}
]]
    local transformed, count = source:gsub("parallel%.waitForAny%s*%b()", replacement, 1)

    if count ~= 1 then
        error("cannot find final parallel.waitForAny(...) call in old startup", 0)
    end

    local env = {
        assert = assert,
        error = error,
        ipairs = ipairs,
        pairs = pairs,
        math = math,
        print = function() end,
        string = string,
        table = table,
        tonumber = tonumber,
        tostring = tostring,
        require = function(name)
            if name == "protocol" or name == "lib.protocol" then
                return protocolStub()
            end

            if name == "pwm" then
                return {
                    set = function() end,
                    get = function() return 0 end,
                    run = function() end,
                }
            end

            error("unsupported require in old startup: " .. tostring(name), 0)
        end,
        rednet = {},
        term = {},
        parallel = {
            waitForAny = function() end,
        },
        sleep = function() end,
    }

    local config = loadWithEnv(transformed, "@" .. path, env)()
    config.display_dt = displayDt or extractDisplayDt(source) or 0.5

    return config
end

local function quoted(value)
    return ("%q"):format(value)
end

local function renderConfig(old)
    local lines = {}

    local function add(line)
        lines[#lines + 1] = line or ""
    end

    add("local config = {}")
    add()
    add("config.modem = {")
    add("    side = " .. quoted(old.modem_side) .. ",")
    add("}")
    add()
    add("config.sync = {")
    add("    enabled = " .. tostring(syncEnabled) .. ",")
    add("    sources = { " .. quoted(source) .. " },")
    add("}")
    add()
    add("config.actuator = {")
    add("    listen = " .. quoted(old.listen) .. ",")
    add(("    display_dt = %.3g,"):format(old.display_dt))
    add("    outputs = {")

    for _, out in ipairs(old.outputs or {}) do
        add("        {")
        add("            blade = " .. tostring(out.blade) .. ",")
        add("            sign = " .. tostring(out.sign) .. ",")
        add("            side = " .. quoted(out.side) .. ",")
        add("        },")
    end

    add("    },")
    add("}")
    add()
    add("return config")
    add()

    return table.concat(lines, "\n")
end

local old = readOldStartup(inputPath)

if not old.modem_side then
    error("old startup did not define MODEM_SIDE", 0)
end

if not old.listen then
    error("old startup did not define LISTEN", 0)
end

if type(old.outputs) ~= "table" then
    error("old startup did not define OUTPUTS", 0)
end

writeFile(outputPath, renderConfig(old))
print("wrote " .. outputPath)
