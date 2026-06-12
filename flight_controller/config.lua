local config = {}

config.runtime = {
    sync = {
        enabled = false,
        sources = { "flight_controller" },
    },

    modem = {
        control = "top",
    },

    input = {
        receive_timeout = 0.1,
        typewriter_name = "linked_typewriter",
    },

    telemetry = {
        broadcast_dt = 0.1,
    },

    data = {
        linear_velocity_dt = 0.1,
    },
}

config.hardware = {
    rotor = {
        modem_side = "front",
        upper_bearing = "swivel_bearing_4",
        lower_bearing = "swivel_bearing_5",
        blade_mount = {
            [1] = 0.0,
            [2] = math.pi / 2,
            [3] = math.pi,
            [4] = 3 * math.pi / 2,
        },
    },
}

config.calibration = {
    sensor_axis = {
        roll = -1,
        pitch = 1,
        yaw = 1,
        yaw_rate = -1,
    },

    rotor = {
        phase_offset_upper = math.pi / 2,
        phase_offset_lower = math.pi / 2,
    },

    mixer_axis = {
        collective = 1,
        roll = -1,
        pitch = 1,
        yaw = -1,
    },
}

config.control = {
    loop_dt = 0.05,
    telemetry_dt = 0.1,
    max_dt = 0.2,
    input_stale_dt = 0.5,

    base_collective = 0.2,
    collective_min = 0.0,
    collective_max = 10.0,

    home_roll = 0.0,
    home_pitch = 0.0,

    max_target_roll = math.rad(30),
    max_target_pitch = math.rad(25),

    roll_target_rate = math.rad(45),
    pitch_target_rate = math.rad(45),
    yaw_target_rate = math.rad(60),
    height_target_rate = 5.0,

    roll_center_rate = math.rad(60),
    pitch_center_rate = math.rad(60),

    yaw_lock_rate_deadband = math.rad(2),

    pid = {
        height = {
            kp = 1.5,
            ki = 0.0,
            kd = 0.0,
            i_min = -3.0,
            i_max = 3.0,
            out_min = -5.0,
            out_max = 5.0,
            deadband = 0.03,
        },

        vertical_speed = {
            kp = 1.0,
            ki = 0.0,
            kd = 0.0,
            i_min = -3.0,
            i_max = 3.0,
            out_min = -6.0,
            out_max = 6.0,
            deadband = 0.03,
        },

        roll = {
            kp = 8.0,
            ki = 4.0,
            kd = 1.0,
            i_min = -2.0,
            i_max = 2.0,
            out_min = -8.0,
            out_max = 8.0,
            deadband = 0.01,
        },

        pitch = {
            kp = 8.0,
            ki = 5.0,
            kd = 3.0,
            i_min = -2.0,
            i_max = 2.0,
            out_min = -12.0,
            out_max = 12.0,
            deadband = 0.01,
        },

        yaw_angle = {
            kp = 3.0,
            ki = 0.0,
            kd = 1.0,
            i_min = -0.5,
            i_max = 0.5,
            out_min = -math.rad(60),
            out_max = math.rad(60),
            deadband = 0.01,
        },

        yaw_rate = {
            kp = 5.0,
            ki = 0.0,
            kd = 0.0,
            i_min = -2.0,
            i_max = 2.0,
            out_min = -8.0,
            out_max = 8.0,
            deadband = 0.01,
        },
    },
}

return config
