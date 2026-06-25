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
    say("Usage: sync [<source> ...] [--logic|--config|--all] [--dry-run] [--quiet]")
    say("       sync --update [--quiet]")
    say("Example: sync flight_controller")
    say("         sync common flight_controller")
    say("Default source: config.sync.sources when config.lua exists")
    say("Default: --logic")
end

local selfUpdate = false
local sourceNames = {}
local mode = "logic"
local modeSet = false
local dryRun = false

for _, arg in ipairs(args) do
    if arg == "--quiet" then
        -- parsed above
    elseif arg == "--dry-run" then
        dryRun = true
    elseif arg == "--update" then
        if selfUpdate or #sourceNames > 0 then
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
        if selfUpdate then
            usage()
            return
        end

        sourceNames[#sourceNames + 1] = arg
    end
end

local function safeSourceName(name)
    return name ~= nil
        and name ~= ""
        and not name:find("/", 1, true)
        and not name:find("\\", 1, true)
        and not name:find("%z")
end

local function syncSources(config)
    local sync = config and config.sync

    if sync == nil then
        return nil
    end

    assert(type(sync.sources) == "table", "config sync.sources must be table")

    return sync.sources
end

local function loadConfigChunk(source, env)
    if setfenv then
        local chunk, err = loadstring(source, "@" .. CONFIG_NAME)

        if not chunk then
            return nil, err
        end

        setfenv(chunk, env)

        return chunk
    end

    return load(source, "@" .. CONFIG_NAME, "t", env)
end

local function partialConfig()
    local file = fs.open(CONFIG_NAME, "r")

    if not file then
        return nil
    end

    local source = file.readAll()
    file.close()

    local patched, count = source:gsub("local%s+config%s*=%s*{%s*}", "config = {}", 1)

    if count ~= 1 then
        return nil
    end

    local env = setmetatable({}, { __index = _G })
    local chunk = loadConfigChunk(patched, env)

    if not chunk then
        return nil
    end

    pcall(chunk)

    return env.config
end

local function defaultSourceNames()
    if not fs.exists(CONFIG_NAME) then
        return nil
    end

    local ok, config = pcall(require, "config")

    if not ok then
        config = partialConfig()
    end

    return syncSources(config)
end

if selfUpdate and (#sourceNames > 0 or modeSet or dryRun) then
    usage()
    return
end

if not selfUpdate and #sourceNames == 0 then
    local defaults = defaultSourceNames()

    if not defaults then
        usage()
        return
    end

    sourceNames = defaults
end

for _, sourceName in ipairs(sourceNames) do
    if not safeSourceName(sourceName) then
        usage()
        return
    end
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

local function encodeRelativePath(value)
    local out = {}

    for segment in value:gmatch("[^/]+") do
        out[#out + 1] = encodePathSegment(segment)
    end

    return table.concat(out, "/")
end

local function pathFromUrl(value)
    local path = value:match("^https?://[^/]+(/.*)$")

    if path then
        return stripQueryAndFragment(path)
    end

    return stripQueryAndFragment(value)
end

local function baseName(path)
    return path:match("[^/]+$") or path
end

local function safeRelativePath(path)
    if not path or path == "" then
        return false
    end

    if path:sub(1, 1) == "/" then
        return false
    end

    if path:find("\\", 1, true) or path:find("%z") then
        return false
    end

    for segment in path:gmatch("[^/]+") do
        if segment == "." or segment == ".." then
            return false
        end
    end

    return not path:find("//", 1, true)
end

local function safeLuaName(name)
    return safeRelativePath(name) and name:sub(-4) == ".lua"
end

local function safeDirName(name)
    return safeRelativePath(name) and name:sub(-1) ~= "/"
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

local protectedDirs = {
    ["rom"] = true,
}

local function protectedName(name)
    return protectedNames[name] or protectedNames[baseName(name)]
end

local function protectedDir(name)
    return protectedDirs[name] or protectedDirs[baseName(name)]
end

local function localPath(name)
    if localDir == "" then
        return name
    end

    return fs.combine(localDir, name)
end

local function listDir(path)
    local ok, files = pcall(fs.list, path)

    if ok then
        return files
    end

    if path == "" then
        ok, files = pcall(fs.list, "/")

        if ok then
            return files
        end
    end

    error("Cannot list local directory " .. tostring(path), 0)
end

local function listLocalLuaFilesAt(dir, prefix, out)
    for _, name in ipairs(listDir(dir)) do
        local path = dir == "" and name or fs.combine(dir, name)
        local relative = prefix == "" and name or (prefix .. "/" .. name)

        if fs.isDir(path) then
            if not protectedDir(relative) then
                listLocalLuaFilesAt(path, relative, out)
            end
        elseif safeLuaName(relative) then
            out[#out + 1] = relative
        end
    end
end

local function listLocalLuaFiles()
    local out = {}
    listLocalLuaFilesAt(localDir, "", out)
    return out
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
    local dir = fs.getDir(path)

    if dir and dir ~= "" and not fs.exists(dir) then
        fs.makeDir(dir)
    end

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

local function hrefToEntry(href, basePath)
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

    if relative == "" then
        return nil
    end

    local isDir = relative:sub(-1) == "/"

    while relative:sub(-1) == "/" do
        relative = relative:sub(1, -2)
    end

    if relative == "" then
        return nil
    end

    if isDir then
        if safeDirName(relative) then
            return {
                name = relative,
                isDir = true,
            }
        end

        return nil
    end

    if safeLuaName(relative) then
        return {
            name = relative,
            isDir = false,
        }
    end

    return nil
end

local function collectEntry(out, seen, href, basePath)
    local entry = hrefToEntry(href, basePath)

    if entry then
        local key = (entry.isDir and "d:" or "f:") .. entry.name

        if not seen[key] then
            seen[key] = true
            out[#out + 1] = entry
        end
    end
end

local function parseRemoteEntries(body, basePath)
    local out = {}
    local seen = {}

    for href in body:gmatch("<[^>]-[Hh][Rr][Ee][Ff][^>]*>(.-)</[^>]-[Hh][Rr][Ee][Ff]>") do
        collectEntry(out, seen, href, basePath)
    end

    for href in body:gmatch("[Hh][Rr][Ee][Ff]%s*=%s*\"([^\"]+)\"") do
        collectEntry(out, seen, href, basePath)
    end

    for href in body:gmatch("[Hh][Rr][Ee][Ff]%s*=%s*'([^']+)'") do
        collectEntry(out, seen, href, basePath)
    end

    table.sort(out, function(a, b)
        if a.isDir ~= b.isDir then
            return a.isDir
        end

        return a.name < b.name
    end)

    return out
end

local function getRemoteEntries(remoteDir, basePath)
    local body = httpRead({
        url = remoteDir,
        method = "PROPFIND",
        headers = { Depth = "1" },
    })

    if body then
        local entries = parseRemoteEntries(body, basePath)

        if #entries > 0 then
            return entries
        end
    end

    local err
    body, err = httpRead({ url = remoteDir })

    if not body then
        error(("Cannot read remote directory %s: %s"):format(remoteDir, tostring(err)), 0)
    end

    return parseRemoteEntries(body, basePath)
end

local function collectRemoteNames(remoteDir, basePath, prefix, out, seen, visited)
    if visited[remoteDir] then
        return
    end

    visited[remoteDir] = true

    for _, entry in ipairs(getRemoteEntries(remoteDir, basePath)) do
        local relative = prefix == "" and entry.name or (prefix .. "/" .. entry.name)

        if entry.isDir then
            local childDir = ensureTrailingSlash(remoteDir .. encodeRelativePath(entry.name))
            local childBasePath = urlDecode(pathFromUrl(childDir))
            collectRemoteNames(childDir, childBasePath, relative, out, seen, visited)
        elseif safeLuaName(relative) and not protectedName(relative) and not seen[relative] then
            seen[relative] = true
            out[#out + 1] = relative
        end
    end
end

local function getRemoteNames(remoteDir, basePath)
    local names = {}

    collectRemoteNames(remoteDir, basePath, "", names, {}, {})

    table.sort(names)

    if #names == 0 then
        error("No remote .lua files found. Stop to avoid deleting local files.", 0)
    end

    return names
end

local function remoteDirForSource(sourceName)
    return ensureTrailingSlash(BASE_URL .. encodePathSegment(sourceName))
end

local function loadSource(sourceName)
    local remoteDir = remoteDirForSource(sourceName)
    local basePath = urlDecode(pathFromUrl(remoteDir))
    local ok, names = pcall(getRemoteNames, remoteDir, basePath)

    if ok then
        return {
            name = sourceName,
            remoteDir = remoteDir,
            names = names,
        }
    end

    error(names, 0)
end

local function loadSources(names)
    local sources = {}

    for _, sourceName in ipairs(names) do
        sources[#sources + 1] = loadSource(sourceName)
    end

    return sources
end

local function downloadFile(remoteDir, name)
    local body, err = httpRead({
        url = remoteDir .. encodeRelativePath(name),
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

local sources = loadSources(sourceNames)
local selectedNames = {}
local skippedNames = {}
local remoteSet = {}
local selectedSource = {}
local sourceLabels = {}

for _, source in ipairs(sources) do
    sourceLabels[#sourceLabels + 1] = source.name

    for _, name in ipairs(source.names) do
        if selectedName(name) then
            if not remoteSet[name] then
                selectedNames[#selectedNames + 1] = name
            end

            remoteSet[name] = true
            selectedSource[name] = source
        else
            skippedNames[#skippedNames + 1] = source.name .. "/" .. name
        end
    end
end

table.sort(selectedNames)
table.sort(skippedNames)

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

say("Sources: " .. table.concat(sourceLabels, " -> "))
say("Local: " .. (localDir == "" and "/" or localDir))
say("Mode: " .. mode)
if dryRun then
    say("Dry run: no local changes will be made")
end

local remoteData = {}

for _, name in ipairs(selectedNames) do
    local source = selectedSource[name]
    local ok, data = pcall(downloadFile, source.remoteDir, name)

    if ok then
        remoteData[name] = data
    else
        fail(data)
    end
end

if #failures > 0 then
    error(("%d sync errors before local changes"):format(#failures), 0)
end

local function newPlan()
    return {
        add = {},
        update = {},
        same = {},
        delete = {},
        skip = {},
        blocked = {},
    }
end

local function addAll(target, source)
    for _, item in ipairs(source) do
        target[#target + 1] = item
    end
end

local function printPlanList(label, list)
    if #list == 0 then
        return
    end

    say(label .. ":")

    for _, item in ipairs(list) do
        say("  " .. item)
    end
end

local function printPlan(plan)
    say("Plan:")
    printPlanList("add", plan.add)
    printPlanList("update", plan.update)
    printPlanList("same", plan.same)
    printPlanList("skip", plan.skip)
    printPlanList("delete stale", plan.delete)
    printPlanList("blocked", plan.blocked)

    if #plan.add == 0
        and #plan.update == 0
        and #plan.same == 0
        and #plan.skip == 0
        and #plan.delete == 0
        and #plan.blocked == 0
    then
        say("  no changes")
    end
end

local function buildPlan()
    local plan = newPlan()
    addAll(plan.skip, skippedNames)

    for _, name in ipairs(listLocalLuaFiles()) do
        local path = localPath(name)

        if shouldDeleteLocal(name) and not protectedName(name) and not fs.isDir(path) and not remoteSet[name] then
            plan.delete[#plan.delete + 1] = name
        end
    end

    for _, name in ipairs(selectedNames) do
        local path = localPath(name)

        if fs.exists(path) and fs.isDir(path) then
            plan.blocked[#plan.blocked + 1] = name .. " is a local directory"
        else
            local data = remoteData[name]
            local old = readFile(path)

            if old == data then
                plan.same[#plan.same + 1] = name
            elseif old == nil then
                plan.add[#plan.add + 1] = name
            else
                plan.update[#plan.update + 1] = name
            end
        end
    end

    return plan
end

local plan = buildPlan()

if dryRun then
    printPlan(plan)
    return
end

for _, name in ipairs(plan.delete) do
    local path = localPath(name)

    local ok, err = pcall(fs.delete, path)

    if ok then
        stats.deleted = stats.deleted + 1
        say("delete " .. name)
    else
        fail(("delete %s failed: %s"):format(name, tostring(err)))
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
