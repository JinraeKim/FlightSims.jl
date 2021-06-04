using FlightSims
const FS = FlightSims


function main()
    power_loop = PowerLoop()
    cg = command_generator(power_loop)
end
