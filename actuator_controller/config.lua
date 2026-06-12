local config = {}

config.modem = {
    side = "top",
}

config.sync = {
    enabled = true,
    target = "actuator_controller",
}

config.actuator = {
    listen = "upper",
    display_dt = 0.5,
    outputs = {
        {
            blade = 1,
            sign = -1,
            side = "back",
        },
        {
            blade = 2,
            sign = 1,
            side = "left",
        },
    },
}

return config
