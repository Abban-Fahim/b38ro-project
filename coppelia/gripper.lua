-- lua

function sysCall_init()
    sim = require('sim')
    ros = require('simROS2')

    jointR1 = sim.getObject("./RIGHT_BOTTOM")
    jointR2 = sim.getObject("./RIGHT_TIP")
    jointL1 = sim.getObject("./LEFT_BOTTOM")
    jointL2 = sim.getObject("./LEFT_TIP")
    
    jointArray = {jointR1, jointR2, jointL1, jointL2}
    open_angles = {50, 12, -50, 12}
    close_angles = {0, 0, 0, 0}

    -- do some initialization here
end

function sysCall_actuation()
    -- put your actuation code here
end

function sysCall_sensing()
    -- put your sensing code here
end

function sysCall_cleanup()
    -- do some clean-up here
end

-- See the user manual or the available code snippets for additional callback functions and details
