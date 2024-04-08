function sysCall_init()
    sim = require('sim')
    ros = require('simROS2')

    rgb_pub = ros.createPublisher("/camera/image/raw", "sensor_msgs/msg/Image")
    ros.publisherTreatUInt8ArrayAsString(rgb_pub)

    -- depthCam=sim.getObject('./depth')
    -- depthView=sim.floatingViewAdd(0.9,0.9,0.2,0.2,0)
    -- sim.adjustView(depthView,depthCam,64)

    colorCam=sim.getObject('./rgb')
    colorView=sim.floatingViewAdd(0.69,0.9,0.2,0.2,0)
    sim.adjustView(colorView,colorCam,64)

end

function sysCall_sensing()
    local img_data, width, height = sim.getVisionSensorCharImage(colorCam)
    ros.publish(rgb_pub, {
        data=img_data,
        width=width,
        height=height,
        encoding="rgb8",
        header={stamp=ros.getTime(),
        frame_id="rgb_cam"},
        step=width*3,
        is_bigendian=1
    })
end
