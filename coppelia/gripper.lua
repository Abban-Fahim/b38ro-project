-- lua

function gripper_cb(msg)

    if msg.data == 0.0 then
        print("opening gripper")
        sim.setObjectParent(attachedShape, -1, true)
    else
        print("closing gripper")
        index=0
        while true do
            shape=sim.getObjects(index,sim.object_shape_type)
            if (shape==-1) then
                break
            end
            if (sim.getObjectInt32Param(shape,sim.shapeintparam_static)==0) and (sim.getObjectInt32Param(shape,sim.shapeintparam_respondable)~=0) and (sim.checkProximitySensor(objectSensor,shape)==1) then
                -- Shape was detected
                attachedShape=shape
                -- Do the "fake" connection:
                sim.setObjectParent(attachedShape,connector,true)
                break
            end
            index=index+1
        end
    end

    sim.setJointTargetPosition(jointR1, 0.8 - msg.data)
    sim.setJointTargetPosition(jointL1, -(0.8 - msg.data))
end

function sysCall_init()
    sim = require('sim')
    ros = require('simROS2')

    ros.createSubscription("/gripper_pose", "std_msgs/msg/Float32", "gripper_cb")

    jointR1 = sim.getObject("./RIGHT_BOTTOM")
    jointR2 = sim.getObject("./RIGHT_TIP")
    jointL1 = sim.getObject("./LEFT_BOTTOM")
    jointL2 = sim.getObject("./LEFT_TIP")

    connector=sim.getObject('./attachPoint')
    objectSensor=sim.getObject('./attachProxSensor')

    attachedShape = 0

-- c) And just before opening the gripper again, detach the previously attached shape:
--
    sim.setObjectParent(attachedShape,-1,true)
    
    jointArray = {jointR1, jointR2, jointL1, jointL2}
    open_angles = {0, 0, 0, 0}
    close_angles = {0, 0, 0, 0}

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
