-- ================================================
-- Remote API program for controlling arm robot
-- @yudarw
-- June 25, 2022
-- ================================================

-- Initialization:
function sysCall_init()
    corout=coroutine.create(coroutineMain)
    
    -- Get object handles:
    forceSensor=sim.getObject('./force_sensor')
    simTip=sim.getObject('./ikTip')
    simTarget=sim.getObject('./ikTarget')
    robot=sim.getObject('.')
    targetPos=sim.getObject('./targetPos')
    gripperHandle=sim.getObject('./RG2')
    
    -- Prepare an ik group:
    ikEnv=simIK.createEnvironment()
    ikGroup=simIK.createIkGroup(ikEnv)
    
    -- Method 1:
    -- simIK.addIkElementFromScene(ikEnv,ikGroup,robot,simTip,simTarget,simIK.constraint_pose)
    
    -- Method 2:
    simIK.setIkGroupCalculation(ikEnv,ikGroup,simIK.method_damped_least_squares,0.01,10)
    local ikElement=simIK.addIkElementFromScene(ikEnv,ikGroup,robot,simTip,simTarget,simIK.constraint_pose)
    simIK.setIkElementPrecision(ikEnv,ikGroup,ikElement,{0.0005,0.005*math.pi/180})
end

function sysCall_actuation()
     if coroutine.status(corout)~='dead' then
        local ok,errorMsg=coroutine.resume(corout)
        if errorMsg then
            error(debug.traceback(corout,errorMsg),2)
        end
    end
end

-- ==================================================
-- RemoteApi Functions
-- ==================================================
-- Set Robot Speed --
function remoteApi_setSpeed(inInt)
    local max_speed=5.0
    local sp_ratio=inInt[1]
    local sp=(sp_ratio/100)*max_speed
    local rot_sp=(sp_ratio/100)*math.pi/2
    ikMaxVel={sp,sp,sp,rot_sp}
end

-- Move Robot Position --
function remoteApi_movePosition(inInt,inFloat,inString,inBuffer)
    targetPos[1]=inFloat[1]
    targetPos[2]=inFloat[2]
    targetPos[3]=inFloat[3]
    targetOri[1]=inFloat[4]
    targetOri[2]=inFloat[5]
    targetOri[3]=inFloat[6]
    moveIkMode=true
end

-- Move Robot Joint Position --
function remoteApi_moveJointPosition(pos)
    for i=1,6,1 do
        jointPos[i] = pos[i] 
    end
    moveJointMode=true
end

-- Get Object Position --
function remoteApi_getPosition()
    --val = sim.getFloatSignal('forceX')
    local pos=sim.getObjectPosition(simTip, robotHandle)
    local rot=sim.getObjectOrientation(simTip, robotHandle)
    local endpos = {pos[1],pos[2],pos[3],rot[1],rot[2],rot[3]}
    return {},endpos,{},''
end

-- Set Gripper --
function remoteApi_setGripper(state)
    velocity=0.11
    force=20
    if state[1]==0 then
        velocity=-velocity
    end
    
    local dat={}
    dat.velocity=velocity
    dat.force=force
    sim.writeCustomDataBlock(gripperHandle,'activity',sim.packTable(dat))
end
-- =======================================================



-- Inverse Kinematic Function:
function moveToPoseCallback(q,velocity,accel,auxData)
    sim.setObjectPose(simTarget,-1,q)
    simIK.applyIkEnvironmentToScene(auxData.ikEnv,auxData.ikGroup)
end

function moveToPose_viaIK(data, pose)
    local pos = {pose[1], pose[2], pose[3]}
    local ori = {pose[4], pose[5], pose[6]}
    sim.setObjectPosition(targetPos, robot, pos)
    sim.setObjectOrientation(targetPos, robot, ori)
    local targetQ  = sim.getObjectPose(targetPos,-1)
    local currentQ =sim.getObjectPose(simTip,-1)
    return sim.moveToPose(-1,currentQ,data.maxVel,data.maxAccel,data.maxJerk,targetQ,moveToPoseCallback,data,nil)
end

-- Forward Kinematic Function:
function moveToConfigCallback(config,velocity,accel,auxData)
    for i=1,#auxData.joints,1 do
        local jh=auxData.joints[i]
        if sim.isDynamicallyEnabled(jh) then
            sim.setJointTargetPosition(jh,config[i])
        else    
            sim.setJointPosition(jh,config[i])
        end
    end
end

function moveToConfig_viaFK(auxData, goalConfig)
    local startConfig={}
    for i=1,#auxData.joints,1 do
        startConfig[i]=sim.getJointPosition(auxData.joints[i])
    end
    sim.moveToConfig(-1,startConfig,nil,nil,auxData.maxVel,auxData.maxAccel,auxData.maxJerk,goalConfig,nil,moveToConfigCallback,auxData,nil)
end


function convertToPose2(pos,rot)
    local pose={}
    pose[1] = pos[1]
    pose[2] = pos[2]
    pose[3] = pos[3]
    pose[4] = rot[1]
    pose[5] = rot[2]
    pose[6] = rot[3]
    return pose 
end


-- Local set position function:
function setPosition(auxData, pos) 
    pose={}
    pose[1]=pos[1] / 1000
    pose[2]=pos[2] / 1000
    pose[3]=pos[3] / 1000
    pose[4]=pos[4] * math.pi / 180
    pose[5]=pos[5] * math.pi / 180
    pose[6]=pos[6] * math.pi / 180
    print(pose)
    moveToPose_viaIK(auxData, pose)
end

function setGripper(state)
    if state then
        remoteApi_setGripper({0})
    else
        remoteApi_setGripper({1})
    end
        
end

-- ==================================================
--                         Main
-- ==================================================
function coroutineMain()

    simJoints={}
    for i=1,6,1 do
        simJoints[i]=sim.getObject('./joint',{index=i-1})
    end
    
    -- FK movement data:
    local initConf={0,0,0,0,0,0}
    local vel=180
    local accel=40
    local jerk=80
    local maxVel={vel*math.pi/180,vel*math.pi/180,vel*math.pi/180,vel*math.pi/180,vel*math.pi/180,vel*math.pi/180}
    local maxAccel={accel*math.pi/180,accel*math.pi/180,accel*math.pi/180,accel*math.pi/180,accel*math.pi/180,accel*math.pi/180}
    local maxJerk={jerk*math.pi/180,jerk*math.pi/180,jerk*math.pi/180,jerk*math.pi/180,jerk*math.pi/180,jerk*math.pi/180}
    local initConf={}
    local maxConfVel={}
    local maxConfAccel={}
    local maxConfJerk={}
    for i=1,#simJoints,1 do
        initConf[i]=sim.getJointPosition(simJoints[i])
        maxConfVel[i]=45*math.pi/180
        maxConfAccel[i]=1.15
        maxConfJerk[i]=0.4
    end
    
    fk_data={}
    fk_data.maxVel=maxConfVel
    fk_data.maxAccel=maxConfAccel
    fk_data.maxJerk=maxConfJerk
    fk_data.joints=simJoints
    

    -- IK movement data:
    local ikMaxVel={1.8,1.8,1.8,4.5}
    local ikMaxAccel={1.8,1.8,1.8,1.24}
    local ikMaxJerk={0.6,0.6,0.6,0.8}
    
    ik_data={}
    ik_data.maxVel=ikMaxVel
    ik_data.maxAccel=ikMaxAccel
    ik_data.maxJerk=ikMaxJerk
    ik_data.ikEnv=ikEnv
    ik_data.ikGroup=ikGroup

    
    -- Test Movement:
    pos1={400, -300, 100, 180, 0, 0}
    pos2={400, -300, 200, 180, 0, 0}
    pos3={400,  300, 200, 180, 0, 0}
    pos4={400,  300, 100, 180, 0, 0}
    
    setGripper(false)
    
    while true do
        moveToConfig_viaFK(fk_data, initConf)
    
        setPosition(ik_data, pos2)
        setPosition(ik_data, pos1)
        setGripper(true)
        sim.wait(1)
        setPosition(ik_data, pos2)
       
        setPosition(ik_data, pos3)
        setPosition(ik_data, pos4)
        setGripper(false)
        sim.wait(1)
        setPosition(ik_data, pos3)
        
    end
    
    sim.wait(5)
    sim.stopSimulation()
   
    
    --while true do
    --    if moveIkMode then
    --        moveIkMode=false
    --        sim.setInt32Signal('moving_status',1)           -- Trigger for C++
    --        sim.setStringSignal('moving_signal', 'MOVING')  -- Trigger for Python
    --        pose = convertToPose2(targetPos, targetOri)
    --        moveToPose_viaIK(ik_data, pose)
    --        sim.setInt32Signal('moving_status',0)
    --        sim.setStringSignal('moving_signal', 'NOT_MOVING')
    --    end
        
    --    if moveJointMode then
    --        moveJointMode=false
    --    end
end
















