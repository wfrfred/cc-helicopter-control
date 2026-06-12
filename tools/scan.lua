while true do
    term.clear()
    term.setCursorPos(1, 1)

    local names = peripheral.getNames()

    print("Peripheral count:", #names)
    print()

    for _, name in ipairs(names) do
        print("== " .. name .. " ==")
        print("type:", table.concat({ peripheral.getType(name) }, ", "))

        local methods = peripheral.getMethods(name)
        for _, method in ipairs(methods) do
            print("  " .. method)
        end

        print()
    end

    sleep(1)
end
