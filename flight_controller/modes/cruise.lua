local common = require("modes.common")

local cruise = {}

local Cruise = {}
Cruise.__index = Cruise

local function horizontalVector(value)
    return vector.new(value.x, 0.0, value.z)
end

function cruise.new()
    return setmetatable({
        velocity = nil,
        manualReleasePending = false,
        toggleHeld = false,
    }, Cruise)
end

function Cruise:clear()
    self.velocity = nil
    self.manualReleasePending = false
end

function Cruise:isActive()
    return self.velocity ~= nil
end

function Cruise:update(input, state, manualActive)
    local cruiseToggle = input.event.cruiseToggle and not self.toggleHeld

    self.toggleHeld = input.event.cruiseToggle == true

    if cruiseToggle then
        self.velocity = horizontalVector(state.world.velocity)
        self.manualReleasePending = manualActive
        return true
    end

    if self.velocity == nil then
        return false
    end

    if self.manualReleasePending then
        if not manualActive then
            self.manualReleasePending = false
        end
        return false
    end

    if manualActive then
        self:clear()
        return true
    end

    return false
end

function Cruise:snapshot()
    if self.velocity == nil then
        return nil
    end

    return horizontalVector(self.velocity)
end

function Cruise:target(input)
    local target = common.base(input)

    target.world.velocity = self:snapshot()

    return target
end

return cruise
