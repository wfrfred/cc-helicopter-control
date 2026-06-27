local sensor_task = {}

--- SableCC sensor boundary contract:
--- - `getLogicalPose()` returns the raw pose sample.
--- - `getLinearVelocity()` returns a world-frame free vector.
--- - `getAngularVelocity()` returns a body-local angular vector.
--- Sensor task publishes samples only; controller state is assembled at the
--- control tick boundary.

local function makeSensors()
    return {
        pose = nil,
        velocity = nil,
        angularVelocity = nil,
    }
end

local function readPose(seq)
    return {
        seq = seq,
        time = os.clock(),
        raw = sublevel.getLogicalPose(),
    }
end

local function readLinearVelocity(seq)
    return {
        seq = seq,
        time = os.clock(),
        world = sublevel.getLinearVelocity(),
    }
end

local function readAngularVelocity(seq)
    return {
        seq = seq,
        time = os.clock(),
        raw = sublevel.getAngularVelocity(),
    }
end

function sensor_task.run(shared)
    shared.sensors = makeSensors()

    local function poseTask()
        local seq = 0

        while shared.running do
            seq = seq + 1
            shared.sensors.pose = readPose(seq)

            sleep(0)
        end
    end

    local function angularVelocityTask()
        local seq = 0

        while shared.running do
            seq = seq + 1
            shared.sensors.angularVelocity = readAngularVelocity(seq)

            sleep(0)
        end
    end

    local function linearVelocityTask()
        local seq = 0

        while shared.running do
            seq = seq + 1
            shared.sensors.velocity = readLinearVelocity(seq)

            sleep(0)
        end
    end

    parallel.waitForAny(poseTask, angularVelocityTask, linearVelocityTask)
end

return sensor_task
