local env = require("tools.test_env")
env.install()

local config = require("config")
local Controller = require("control.controller")
local control_state = require("app.control_state")
local frames = require("lib.frames")
local common = require("modes.common")
local horizontal = require("control.horizontal")
local lock = require("modes.lock")
local navigation = require("modes.navigation")

local function assertClose(name, actual, expected, tolerance)
    tolerance = tolerance or 1.0e-9
    assert(
        math.abs(actual - expected) <= tolerance,
        string.format("%s expected %.12f got %.12f", name, expected, actual)
    )
end

local function rawPoseFromBodyFrame(position, bodyFrame)
    local basis = bodyFrame:basis()
    local rawFrame = frames.fromBasis({
        forward = -basis.right,
        right = -basis.down,
        down = basis.forward,
    }, position)

    return {
        position = position,
        orientation = rawFrame.qWorldFromLocal,
    }
end

local function stateFrom(options)
    options = options or {}

    local position = options.position or vector.new(0.0, 80.0, 0.0)
    local velocity = options.velocity or vector.new(0.0, 0.0, 0.0)
    local angularVelocity = options.angularVelocity or vector.new(0.0, 0.0, 0.0)
    local bodyFrame = frames.bodyFromAngles(
        options.roll or 0.0,
        options.pitch or 0.0,
        options.heading or 0.0,
        position
    )

    return control_state.fromSensors({
        pose = {
            seq = 1,
            time = 0.0,
            raw = rawPoseFromBodyFrame(position, bodyFrame),
        },
        velocity = {
            seq = 1,
            time = 0.0,
            world = velocity,
        },
        angularVelocity = {
            seq = 1,
            time = 0.0,
            raw = angularVelocity,
        },
    }, {
        bodyAxis = config.calibration.body_axis,
    })
end

local controller = Controller.new(config.control)
local controllerTarget = common.target("attitude")

controllerTarget.altitude.position = 0.0
controllerTarget.horizontal.angle.roll = 0.0
controllerTarget.horizontal.angle.pitch = 0.0
controllerTarget.yaw.angle = 0.0

local control = controller:update({
    state = stateFrom(),
    target = controllerTarget,
    dt = config.control.loop.dt,
})
local command = control.output
local controlTerms = control.terms
local oldYawPriority = "yaw" .. "_priority"
local oldYawPriorityTerm = "yaw" .. "Priority"
local oldReducedOrientation = "reduced" .. "Orientation"
local oldReducedFrameHelper = "reduced" .. "FrameFromTargetDown"

assert(config.control.heading[oldYawPriority] == nil, "heading yaw-priority config should be removed")
assert(controlTerms.attitude.target == nil, "attitude terms should not expose split target wrapper")
assert(controlTerms.attitude[oldYawPriorityTerm] == nil, "attitude terms should not expose yaw-priority")
assert(controlTerms.attitude[oldReducedOrientation] == nil, "attitude terms should not expose reduced orientation")
assert(frames[oldReducedFrameHelper] == nil, "reduced target frame helper should be removed")

assertClose("neutral collective", command.collective, 1.0)
assertClose("neutral roll", command.roll, -0.03)
assertClose("neutral pitch", command.pitch, -0.33)
assertClose("neutral yaw", command.yaw, 0.0)

local hold = horizontal.new(config.control)
local holdTarget = common.target("position").horizontal

holdTarget.position.forward = 0.0
holdTarget.position.right = 0.0

local holdResult = hold:update(
    {
        position = {
            forward = 0.0,
            right = 0.0,
        },
        velocity = {
            forward = 0.0,
            right = 0.0,
        },
    },
    {
        position = holdTarget.position,
    },
    holdTarget.feedforward,
    config.control.loop.dt
)
assertClose("position_hold roll", holdResult.output.roll, 0.0)
assertClose("position_hold pitch", holdResult.output.pitch, 0.0)

local heightLock = lock.new({
    initial = 80.0,
    target_rate = config.control.vertical.target_rate,
    rate_deadband = config.control.vertical.lock.speed_deadband,
})
local lockResult = heightLock:update({
    input = 1.0,
    value = 80.0,
    rate = 0.0,
    dt = config.control.loop.dt,
})
assert(lockResult.locked == false, "manual climb should disable height lock feedback")
assert(lockResult.manual == true, "manual climb should be marked as manual lock input")
assert(lockResult.active == nil, "lock result should not expose old active field")
assert(lockResult.pending == nil, "lock result should not expose pending state")
assert(lockResult.source == nil, "lock result should not expose display source")
assertClose("manual climb commanded rate", lockResult.rate, config.control.vertical.target_rate)

local navState = stateFrom({
    position = vector.new(-213, 90, 304),
})
local navMode = navigation.new(config.navigation)
navMode:enter({
    command = { action = "activate", waypoint = "home" },
    state = navState,
    dt = config.control.loop.dt,
})
assert(navMode:update({
    input = {},
    state = navState,
    dt = config.control.loop.dt,
}).terms.active == true, "navigation should activate configured home waypoint")

print("smoke ok")
