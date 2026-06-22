local M = {}

local cases = {
    {
        name = "manual neutral",
        input = {
            roll = 0.0,
            pitch = 0.0,
            climb = 0.0,
            heading = 0.0,
        },
        expected = {
            mode = "manual",
            target = {
                roll = 0.0,
                pitch = 0.0,
                height = 80.0,
                verticalSpeed = 0.0,
                heightActive = true,
            },
            command = {
                collective = 1.0,
                roll = -0.0467649,
                pitch = -0.33031404,
                yaw = 0.00813396,
            },
        },
    },
    {
        name = "manual roll positive",
        input = {
            roll = 1.0,
            pitch = 0.0,
            climb = 0.0,
            heading = 0.0,
        },
        expected = {
            mode = "manual",
            target = {
                roll = 0.039269908169872414,
                pitch = 0.0,
                height = 80.0,
                verticalSpeed = 0.0,
                heightActive = true,
            },
            command = {
                collective = 1.0,
                roll = 0.36258216834348878,
                pitch = -0.34653929259651811,
                yaw = -0.12665749530720738,
            },
        },
    },
    {
        name = "manual pitch positive",
        input = {
            roll = 0.0,
            pitch = 1.0,
            climb = 0.0,
            heading = 0.0,
        },
        expected = {
            mode = "manual",
            target = {
                roll = 0.0,
                pitch = 0.039269908169872414,
                height = 80.0,
                verticalSpeed = 0.0,
                heightActive = true,
            },
            command = {
                collective = 1.0,
                roll = -0.017373584939957373,
                pitch = 0.2540603450491003,
                yaw = 0.011262440799463147,
            },
        },
    },
    {
        name = "manual climb positive",
        input = {
            roll = 0.0,
            pitch = 0.0,
            climb = 1.0,
            heading = 0.0,
        },
        expected = {
            mode = "manual",
            target = {
                roll = 0.0,
                pitch = 0.0,
                height = 80.0,
                verticalSpeed = 5.0,
                heightActive = false,
            },
            command = {
                collective = 6.0,
                roll = -0.0467649,
                pitch = -0.33031404,
                yaw = 0.00813396,
            },
        },
    },
    {
        name = "manual heading positive",
        input = {
            roll = 0.0,
            pitch = 0.0,
            climb = 0.0,
            heading = 1.0,
        },
        expected = {
            mode = "manual",
            target = {
                roll = 0.0,
                pitch = 0.0,
                height = 80.0,
                verticalSpeed = 0.0,
                heightActive = true,
            },
            command = {
                collective = 1.0,
                roll = -0.069304470296973508,
                pitch = -0.27322955718493774,
                yaw = 3.7631040288198294,
            },
        },
    },
    {
        name = "position_hold neutral",
        expected = {
            mode = "position_hold",
            positionHoldActive = true,
            target = {
                roll = 0.0,
                pitch = 0.0,
            },
        },
    },
    {
        name = "cruise capture",
        expected = {
            mode = "cruise",
            cruiseVelocity = {
                x = 3.0,
                z = -1.0,
            },
            target = {
                roll = 0.052500000000000005,
                pitch = -0.0155,
            },
        },
    },
    {
        name = "navigation active target",
        expected = {
            mode = "navigation",
            active = true,
            phase = "climb",
            target = {
                x = -213.0,
                z = 304.0,
                height = 100.0,
                heading = 0.0,
            },
        },
    },
    {
        name = "input stale zero input behavior",
        inputAge = 0.6,
        expected = {
            stale = true,
            manualAxes = {
                roll = 0.0,
                pitch = 0.0,
                climb = 0.0,
                heading = 0.0,
            },
        },
    },
}

function M.cases()
    return cases
end

return M
