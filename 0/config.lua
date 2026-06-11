local config = {}

config.modem = {
    side = "top",
}

config.sync = {
    enabled = true,
    target = "0",
}

config.displays = {
    main = nil,
    attitude = nil,
}

config.input = {
    send_dt = 0.05,
    typewriter_name = "linked_typewriter",
}

config.telemetry = {
    receive_timeout = 0.1,
}

config.monitor = {
    text_scale = 0.5,
    draw_dt = 0.1,
}

config.attitude = {
    text_scale = 0.5,
    draw_dt = 0.05,
    pitch_deg_per_row = 7.0,
    cell_aspect = 0.55,
    roll_offset_deg = 0.0,
    pitch_offset_deg = 0.0,
    roll_limit_deg = 75.0,
    pitch_limit_deg = 35.0,
    center_marker = "--+--",
}

return config
