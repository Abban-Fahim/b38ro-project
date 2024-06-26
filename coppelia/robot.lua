-- lua

function movCallback(config,vel,accel,handles)
    for i=1,#handles,1 do

        if sim.isDynamicallyEnabled(handles[i]) then
            sim.setJointTargetPosition(handles[i],config[i])
        else    
            sim.setJointPosition(handles[i],config[i])
        end
    end
end

function lol(handles,maxVel,maxAccel,maxJerk,targetConf)
    local currentConf={}
    for i=1,#handles,1 do
        currentConf[i]=sim.getJointPosition(handles[i])
    end
    sim.moveToConfig(-1,currentConf,nil,nil,maxVel,maxAccel,maxJerk,targetConf,nil,movCallback,handles)
end

function angle_cb(msg)
    local jointHandles={}
    for i=1,6,1 do
        print(i)
        jointHandles[i]=sim.getObject('./J'..(i-1))
    end
    
    local vel=35  
    local accel=10
    local jerk=5
    local maxVel={vel*math.pi/180,vel*math.pi/180,vel*math.pi/180,vel*math.pi/180,vel*math.pi/180,vel*math.pi/180}
    local maxAccel={accel*math.pi/180,accel*math.pi/180,accel*math.pi/180,accel*math.pi/180,accel*math.pi/180,accel*math.pi/180}
    local maxJerk={jerk*math.pi/180,jerk*math.pi/180,jerk*math.pi/180,jerk*math.pi/180,jerk*math.pi/180,jerk*math.pi/180}
    print(msg)
    lol(jointHandles,maxVel,maxAccel,maxJerk,msg.points[1].positions)
    
    -- print(msg.data)
end

function sysCall_init()
    sim = require('sim')
    ros = require('simROS2')

    -- code for tetsing an IK environment, only to proof our Jacobian matrices with coppelia's
    -- simIK = require("simIK")
    -- ikEnv=simIK.createEnvironment()
    -- ikGroup=simIK.createGroup(ikEnv)
    -- tip = sim.getObject("./tip")
    -- target = sim.getObject("./target")
    -- -- targe = sim.getObject("./Sphere")
    -- simIK.addElementFromScene(ikEnv,ikGroup,-1,tip,target,simIK.constraint_pose)
    -- simIK.handleGroup(ikEnv,ikGroup,{syncWorlds=true,allowError=true})
    
    ros.createSubscription('/joint_trajectory_controller/joint_trajectory', 'trajectory_msgs/msg/JointTrajectory', 'angle_cb')
    -- jacPub = ros.cretePublisher('/jacob_matrix','geometry_msgs/msg/Pose')
    statePub = ros.createPublisher('/joint_states', 'sensor_msgs/msg/JointState')
    
end

function sysCall_actuation()

    local jointNames = {}
    local jointAngles = {}
    local jointVelocities = {}
    for i=1,6,1 do
        jointAngles[i]=sim.getJointPosition(sim.getObject('./J'..(i-1)))
        jointVelocities[i]=sim.getJointVelocity(sim.getObject('./J'..(i-1)))
        jointNames[i]='joint_'..i
    end
    -- print(jointAngles)
    ros.publish(statePub, {name=jointNames, position=jointAngles, velocity=jointVelocities})
end

function sysCall_sensing()
    -- put your sensing code here
end

function sysCall_cleanup()
    -- do some clean-up here
end
