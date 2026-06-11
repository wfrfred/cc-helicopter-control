local args = { ... }

local BASE_URL = ""
local REQUEST_TIMEOUT = 15
local CONFIG_NAME = "config.lua"
local quiet = false

for _, arg in ipairs(args) do
    if arg == "--quiet" then
        quiet = true
    end
end

local function say(...)
    if not quiet then
        print(...)
    end
end

local function usage()
    say("Usage: sync <target> [--logic|--config|--all] [--quiet]")
    say("       sync --update [--quiet]")
    say("Example: sync 0")
    say("Default: --logic")
end

local selfUpdate = false
local targetName = nil
local mode = "logic"
local modeSet = false

for _, arg in ipairs(args) do
    if arg == "--quiet" then
        -- parsed above
    elseif arg == "--update" then
        if selfUpdate or targetName then
            usage()
            return
        end

        selfUpdate = true
    elseif arg == "--logic" or arg == "--config" or arg == "--all" then
        if modeSet then
            usage()
            return
        end

        mode = arg:sub(3)
        modeSet = true
    elseif arg:sub(1, 2) == "--" then
        usage()
        return
    else
        if selfUpdate or targetName then
            usage()
            return
        end

        targetName = arg
    end
end

local function safeTargetName(name)
    return name ~= nil
        and name ~= ""
        and not name:find("/", 1, true)
        and not name:find("\\", 1, true)
        and not name:find("%z")
end

if selfUpdate and (targetName or modeSet) then
    usage()
    return
end

if not selfUpdate and not safeTargetName(targetName) then
    usage()
    return
end

if not http or not http.get then
    error("HTTP API is not available. Enable http and allow 10.81.28.121 in CC config.", 0)
end

local function trim(value)
    return (value:gsub("^%s+", ""):gsub("%s+$", ""))
end

local function ensureTrailingSlash(value)
    if value:sub(-1) == "/" then
        return value
    end

    return value .. "/"
end

local function stripQueryAndFragment(value)
    return (value:gsub("[#?].*$", ""))
end

local function xmlDecode(value)
    value = value:gsub("&lt;", "<")
    value = value:gsub("&gt;", ">")
    value = value:gsub("&quot;", "\"")
    value = value:gsub("&apos;", "'")
    value = value:gsub("&amp;", "&")

    return value
end

local function urlDecode(value)
    local decoded = value:gsub("%%(%x%x)", function(hex)
        return string.char(tonumber(hex, 16))
    end)

    return decoded
end

local function encodePathSegment(value)
    local encoded = value:gsub("[^A-Za-z0-9%-%._~]", function(char)
        return ("%%%02X"):format(char:byte())
    end)

    return encoded
end

local function pathFromUrl(value)
    local path = value:match("^https?://[^/]+(/.*)$")

    if path then
        return stripQueryAndFragment(path)
    end

    return stripQueryAndFragment(value)
end

local function safeLuaName(name)
    if not name or name == "" then
        return false
    end

    if name:find("/", 1, true) or name:find("\\", 1, true) or name:find("%z") then
        return false
    end

    return name:sub(-4) == ".lua"
end

local function currentProgramPath()
    if shell and shell.getRunningProgram then
        local program = shell.getRunningProgram()

        if shell.resolve then
            return shell.resolve(program)
        end

        return program
    end

    return "sync.lua"
end

local selfPath = currentProgramPath()
local selfName = fs.getName(selfPath)
local localDir = fs.getDir(selfPath)
local protectedNames = {
    [selfName] = true,
    ["sync.lua"] = true,
}

local function localPath(name)
    if localDir == "" then
        return name
    end

    return fs.combine(localDir, name)
end

local function listLocalDir()
    local ok, files = pcall(fs.list, localDir)

    if ok then
        return files
    end

    if localDir == "" then
        ok, files = pcall(fs.list, "/")

        if ok then
            return files
        end
    end

    error("Cannot list local directory " .. tostring(localDir), 0)
end

local function openFile(path, mode)
    local ok, handle, err = pcall(fs.open, path, mode)

    if ok then
        return handle, err
    end

    if mode:find("b", 1, true) then
        local textMode = mode:gsub("b", "")
        ok, handle, err = pcall(fs.open, path, textMode)

        if ok then
            return handle, err
        end
    end

    return nil, handle
end

local function readFile(path)
    local file = openFile(path, "rb")

    if not file then
        return nil
    end

    local data = file.readAll() or ""
    file.close()

    return data
end

local function writeFile(path, data)
    local file, err = openFile(path, "wb")

    if not file then
        error(("Cannot open %s for writing: %s"):format(path, tostring(err)), 0)
    end

    file.write(data)
    file.close()
end

local function httpRead(options)
    options.timeout = options.timeout or REQUEST_TIMEOUT

    local ok, handle, err, failedHandle = pcall(http.get, options)

    if not ok then
        return nil, handle
    end

    if not handle then
        local code

        if failedHandle and failedHandle.getResponseCode then
            code = failedHandle.getResponseCode()
        end

        if failedHandle and failedHandle.close then
            failedHandle.close()
        end

        if code then
            return nil, ("%s (HTTP %s)"):format(tostring(err), tostring(code))
        end

        return nil, tostring(err)
    end

    local body = handle.readAll() or ""
    handle.close()

    return body
end

local function hrefToName(href, basePath)
    href = trim(xmlDecode(href))

    if href == "" then
        return nil
    end

    local path = urlDecode(pathFromUrl(href))
    local relative

    if path:sub(1, #basePath) == basePath then
        relative = path:sub(#basePath + 1)
    elseif path:sub(1, 1) ~= "/" then
        relative = path:gsub("^%./", "")
    else
        return nil
    end

    if relative == "" or relative:find("/", 1, true) then
        return nil
    end

    if not safeLuaName(relative) then
        return nil
    end

    return relative
end

local function collectName(out, seen, href, basePath)
    local name = hrefToName(href, basePath)

    if name and not protectedNames[name] and not seen[name] then
        seen[name] = true
        out[#out + 1] = name
    end
end

local function parseRemoteNames(body, basePath)
    local out = {}
    local seen = {}

    for href in body:gmatch("<[^>]-[Hh][Rr][Ee][Ff][^>]*>(.-)</[^>]-[Hh][Rr][Ee][Ff]>") do
        collectName(out, seen, href, basePath)
    end

    for href in body:gmatch("[Hh][Rr][Ee][Ff]%s*=%s*\"([^\"]+)\"") do
        collectName(out, seen, href, basePath)
    end

    for href in body:gmatch("[Hh][Rr][Ee][Ff]%s*=%s*'([^']+)'") do
        collectName(out, seen, href, basePath)
    end

    table.sort(out)

    return out
end

local function getRemoteNames(remoteDir, basePath)
    local body = httpRead({
        url = remoteDir,
        method = "PROPFIND",
        headers = { Depth = "1" },
    })

    if body then
        local names = parseRemoteNames(body, basePath)

        if #names > 0 then
            return names
        end
    end

    local err
    body, err = httpRead({ url = remoteDir })

    if not body then
        error(("Cannot read remote directory %s: %s"):format(remoteDir, tostring(err)), 0)
    end

    local names = parseRemoteNames(body, basePath)

    if #names == 0 then
        error("No remote .lua files found. Stop to avoid deleting local files.", 0)
    end

    return names
end

local function downloadFile(remoteDir, name)
    local body, err = httpRead({
        url = remoteDir .. encodePathSegment(name),
        binary = true,
    })

    if not body then
        error(("Cannot download %s: %s"):format(name, tostring(err)), 0)
    end

    return body
end

local function updateSelf()
    local remoteRoot = ensureTrailingSlash(BASE_URL)
    local data = downloadFile(remoteRoot, "sync.lua")
    local old = readFile(selfPath)

    if old == data then
        say("sync.lua already current")
        return
    end

    writeFile(selfPath, data)
    say("sync.lua updated")
end

if selfUpdate then
    updateSelf()
    return
end

local function selectedName(name)
    if mode == "all" then
        return true
    end

    if mode == "config" then
        return name == CONFIG_NAME
    end

    return name ~= CONFIG_NAME
end

local function shouldDeleteLocal(name)
    if mode == "config" then
        return false
    end

    return selectedName(name)
end

local remoteDir = ensureTrailingSlash(BASE_URL .. encodePathSegment(targetName))
local basePath = urlDecode(pathFromUrl(remoteDir))
local remoteNames = getRemoteNames(remoteDir, basePath)
local selectedNames = {}
local remoteSet = {}

for _, name in ipairs(remoteNames) do
    if selectedName(name) then
        selectedNames[#selectedNames + 1] = name
        remoteSet[name] = true
    end
end

if #selectedNames == 0 then
    if mode == "config" then
        error("Remote config.lua not found.", 0)
    end

    error("No remote files selected. Stop to avoid deleting local files.", 0)
end

local stats = {
    deleted = 0,
    downloaded = 0,
    updated = 0,
    unchanged = 0,
}

local failures = {}

local function fail(message)
    failures[#failures + 1] = message
    say("ERROR " .. message)
end

say("Remote: " .. remoteDir)
say("Local: " .. (localDir == "" and "/" or localDir))
say("Mode: " .. mode)

local remoteData = {}

for _, name in ipairs(selectedNames) do
    local ok, data = pcall(downloadFile, remoteDir, name)

    if ok then
        remoteData[name] = data
    else
        fail(data)
    end
end

if #failures > 0 then
    error(("%d sync errors before local changes"):format(#failures), 0)
end

for _, name in ipairs(listLocalDir()) do
    local path = localPath(name)

    if safeLuaName(name) and shouldDeleteLocal(name) and not protectedNames[name] and not fs.isDir(path) and not remoteSet[name] then
        local ok, err = pcall(fs.delete, path)

        if ok then
            stats.deleted = stats.deleted + 1
            say("delete " .. name)
        else
            fail(("delete %s failed: %s"):format(name, tostring(err)))
        end
    end
end

for _, name in ipairs(selectedNames) do
    local path = localPath(name)

    if fs.exists(path) and fs.isDir(path) then
        fail(name .. " is a local directory")
    else
        local data = remoteData[name]
        local old = readFile(path)

        if old == data then
            stats.unchanged = stats.unchanged + 1
            say("same   " .. name)
        else
            local writeOk, writeErr = pcall(writeFile, path, data)

            if writeOk then
                if old == nil then
                    stats.downloaded = stats.downloaded + 1
                    say("add    " .. name)
                else
                    stats.updated = stats.updated + 1
                    say("update " .. name)
                end
            else
                fail(("write %s failed: %s"):format(name, tostring(writeErr)))
            end
        end
    end
end

say(
    ("Done: %d added, %d updated, %d deleted, %d unchanged"):format(
        stats.downloaded,
        stats.updated,
        stats.deleted,
        stats.unchanged
    )
)

if #failures > 0 then
    error(("%d sync errors"):format(#failures), 0)
end
