----------------------------------------------------------------------------------
-------------------------------- POPUP SECTION BELOW -----------------------------
----------------------------------------------------------------------------------
function uiSetup()
    xml = '<ui title="R2 Robot Arm Control" closeable="false" resizeable="true" activate="false">' .. [[
                    <button text="Print Joint Angles" on-click="printJointAngles" id="1"/>
                    <button text="Print Target State" on-click="printTargetPosition" id="2"/>
                    <button text="Open Gripper" on-click="actuateGripperUI" id="1001" />
                    <button text="Close Gripper" on-click="actuateGripperUI" id="1002" />
                    <label text="" style="* {margin-left: 200px;}"/>        
                    </ui>
                    ]]

    uii=simUI.create(xml)
end

function printJointAngles()
    local jointHandles={}
    for i=1,6,1 do
        jointHandles[i]=sim.getObjectHandle('NiryoOneJoint'..i)
    end

    local currentConf={}
    for i=1,#jointHandles,1 do
        currentConf[i]=sim.getJointPosition(jointHandles[i])
    end
    print(currentConf)
end

function printTargetPosition()
    target = sim.getObjectHandle("nodeIK_obj#0") -- CHANGE FOR THE APPROPRIATE TARGET!
    
    -- get position and orientation of target
    pos = sim.getObjectPosition(target,-1)
    orientation = sim.getObjectOrientation(target,-1)

    -- print to console window
    print(pos, orientation)
end

function actuateGripperUI(ui, id)
    if id == 1001 then
        -- open gripper
        sim.clearIntegerSignal(gripperName.. '_close')
    else
        -- close gripper
        sim.setIntegerSignal(gripperName.. '_close', 1)
    end
end

----------------------------------------------------------------------------------
-------------------------------- SETUP SECTION BELOW -----------------------------
----------------------------------------------------------------------------------
function sysCall_init()
    corout=coroutine.create(coroutineMain)  -- create main coroutine

    -- call UI box setup
    uiSetup()

    -- setup messaging between robots
    messageSetup()

    -- Get gripper name and set as global variable
    local connection=sim.getObjectHandle('NiryoOne_connection')
    local gripper=sim.getObjectChild(connection,0)
    gripperName = "NiryoNoGripper"
    if gripper~=-1 then
        gripperName=sim.getObjectName(gripper)
    end
end

function sysCall_actuation()
    if coroutine.status(corout)~='dead' then
        local ok,errorMsg=coroutine.resume(corout)
        if errorMsg then
            error(debug.traceback(corout,errorMsg),2)
        end
    end
end

function messageSetup()
    -- Store global name of me
    myhandle = sim.getObjectHandle(sim.handle_self)
    myname = sim.getObjectName(myhandle)

    -- Setup sending and recieving globally accessible messages
    channel1 = myname.."_CH1"
    channel2 = myname .."_CH2"

    -- Send messages on channels
    sim.setIntegerSignal(channel1,0)
    sim.setIntegerSignal(channel2,0)

    -- Publish my name so others can access it
    sim.setStringSignal("R2", sim.packTable({myname})) -- change this to the correct robot!
    
    -- Print for debugging
    print("R1: " .. channel1)
    print("R1: " .. channel2)
end
----------------------------------------------------------------------------------
------------------------------ ACTUATION SECTION BELOW ---------------------------
----------------------------------------------------------------------------------
function moveToPoseCallback(q,velocity,accel,auxData)
    sim.setObjectPose(auxData.target,-1,q)
    simIK.applyIkEnvironmentToScene(auxData.ikEnv,auxData.ikGroup)
end

function moveToPose_viaIK(maxVelocity,maxAcceleration,maxJerk,targetQ,auxData)
    local currentQ=sim.getObjectPose(auxData.tip,-1)
    return sim.moveToPose(-1,currentQ,maxVelocity,maxAcceleration,maxJerk,targetQ,moveToPoseCallback,auxData,nil)
end

function moveToConfigCallback(config,velocity,accel,auxData)
    for i=1,#auxData.joints,1 do
        local jh=auxData.joints[i]
        if sim.getJointMode(jh)==sim.jointmode_force and sim.isDynamicallyEnabled(jh) then
            sim.setJointTargetPosition(jh,config[i])
        else    
            sim.setJointPosition(jh,config[i])
        end
    end
end

function moveToConfig_viaFK(maxVelocity,maxAcceleration,maxJerk,goalConfig,auxData)
    local startConfig={}
    for i=1,#auxData.joints,1 do
        startConfig[i]=sim.getJointPosition(auxData.joints[i])
    end
    sim.moveToConfig(-1,startConfig,nil,nil,maxVelocity,maxAcceleration,maxJerk,goalConfig,nil,moveToConfigCallback,auxData,nil)
end

function moveToConfig_dXYZ(ikMaxVel,ikMaxAccel,ikMaxJerk,data,mytarget,dX,dY,dZ,qx,qy,qz,qw)
    -- get target handle
    local pose = sim.getObjectPose(mytarget,-1)

    -- adjust current pose to user defined pose
    pose[1] = pose[1] + dX  -- global x
    pose[2] = pose[2] + dY  -- global y
    pose[3] = pose[3] + dZ  -- global z

    pose[4] = pose[4] + qx  -- global x
    pose[5] = pose[5] + qy  -- global y
    pose[6] = pose[6] + qz  -- global z
    pose[7] = pose[7] + qw  -- global z

    moveToPose_viaIK(ikMaxVel,ikMaxAccel,ikMaxJerk,pose,data)
end

function actuateGripper(command)
    if command == "open" then
        -- open gripper
        sim.clearIntegerSignal(gripperName.. '_close')
    elseif command == "close" then
        -- close gripper
        sim.setIntegerSignal(gripperName.. '_close', 1)
    end
    sim.wait(6) -- gripper time to open/ close
end

----------------------------------------------------------------------------------
-------------------------------- MAIN SECTION BELOW ------------------------------
----------------------------------------------------------------------------------
function coroutineMain()
    -- Initialize some values:
    local simJoints={}
    for i=1,6,1 do
        simJoints[i]=sim.getObjectHandle('NiryoOneJoint'..i)
    end
    local simTip=sim.getObjectHandle('nodeIK_obj#0')
    local simTarget=sim.getObjectHandle('referenceIK_obj#0')
    local modelBase=sim.getObjectHandle(sim.handle_self)

    -- create inverse kinematic environment
    ikEnv=simIK.createEnvironment()

    -- Prepare the ik group, using the convenience function 'simIK.addIkElementFromScene':
    ikGroup=simIK.createIkGroup(ikEnv)
    simIK.addIkElementFromScene(ikEnv,ikGroup,modelBase,simTip,simTarget,simIK.constraint_pose)

    -- FK movement data:
    local initConf={0,0,0,0,0,0}
    local vel=180
    local accel=40
    local jerk=80
    local maxVel={vel*math.pi/180,vel*math.pi/180,vel*math.pi/180,vel*math.pi/180,vel*math.pi/180,vel*math.pi/180}
    local maxAccel={accel*math.pi/180,accel*math.pi/180,accel*math.pi/180,accel*math.pi/180,accel*math.pi/180,accel*math.pi/180}
    local maxJerk={jerk*math.pi/180,jerk*math.pi/180,jerk*math.pi/180,jerk*math.pi/180,jerk*math.pi/180,jerk*math.pi/180}

    -- IK movement data:
    local ikMaxVel={0.4,0.4,0.4,1.8}
    local ikMaxAccel={0.8,0.8,0.8,0.9}
    local ikMaxJerk={0.6,0.6,0.6,0.8}

    -- IK delacate movement
    local ikMaxVel_small={0.1,0.1,0.1,0.5}
    local ikMaxAccel_small={0.1,0.1,0.1,0.3}
    local ikMaxJerk_small={0.3,0.3,0.3,0.4}

    -- define FK movements which will be conducted in main actions (note these will be looped through)
    local startConfig={0.018514279276133, 0.53788828849792, -0.89212203025818, 0.0064570941030979, -1.2166624069214, -0.00010122731328011}
    local macro_moves = {{0.0022422843612731, -0.4016324877739, 0.7152601480484, 0.00019060660270043, -1.7453393936157, 0.0022931143175811},
                            {-1.7891311645508, -0.41722574830055, -0.032950282096863, -1.6671049594879, -1.3744693994522, -1.1116399765015},
                            {-2.0959885120392, -0.59904563426971, 0.23584520816803, -1.7741186618805, -1.0830663442612, -1.1563498973846}}

    local data={}
    data.ikEnv=ikEnv
    data.ikGroup=ikGroup
    data.tip=simTip
    data.target=simTarget
    data.joints=simJoints

    ----------------------------------------------------------------
    -- GET ALL OTHER ROBOT NAMES IN CHAIN
    sim.waitForSignal("R4") -- wait for final robot to init
    robot_names = {}
    sig_names = {"R1", "R2", "R3", "R4"}
    for i=1, 2, 1 do
        local theirName = sim.unpackTable( sim.getStringSignal(sig_names[i]) )
        robot_names[i] = theirName[1]
    end

    ----------------------------------------------------------------
    -- MAIN LOOP
    wait_time = 4 -- time to wait between movements (seconds)
    
    --[[
    -- use the fuel rod dummy variable to move to location rather than computer vision
    local fueltip=sim.getObjectHandle(fuelRods[i])
    local poseTarget=sim.getObjectPose(fueltip,-1)
    poseTarget[3]=poseTarget[3]+0.12
    moveToPose_viaIK(ikMaxVel,ikMaxAccel,ikMaxJerk,poseTarget,data)
    sim.wait(wait_time)
    ]]

    -- move to niryo 1 pickup location
    moveToConfig_viaFK(maxVel,maxAccel,maxJerk,startConfig,data)
    local nextPos =  {-0.53273659944534, -0.62394535541534, 0.19894213974476, -1.7880934476852, -1.0828808546066, 0.44577288627625}
    moveToConfig_viaFK(maxVel,maxAccel,maxJerk,nextPos,data)
    
    -- Wait for signal from R1 (rod in place)
    while (sim.getIntegerSignal(robot_names[1].."_CH1") ~= 1) do
        sim.wait(0.25)
    end

    actuateGripper("close") -- close gripper

    -- tell R1 that R2 has closed gripper
    sim.setIntegerSignal(channel1,1)
    sim.wait(2)

    ----------------------------------------------------------------
    -- R2 MOVES TO R3 TO TRANSFER ROD

    -- reset signal to 0 for next fuel rod
    sim.setIntegerSignal(channel1,0)

    local nextPos =  {{-0.49697303771973, -0.61107516288757, 0.20322360098362, -1.7530732154846, -1.1088200807571, 0.40732210874557},
                     {-0.71575927734375, -0.19643351435661, -0.37149202823639, -1.9780020713806, -0.97086751461029, 0.6506695151329}}
    
    -- move to next robot
    for movestep = 1, 2 do
        moveToConfig_viaFK(maxVel,maxAccel,maxJerk,nextPos[movestep],data)
        --sim.wait(wait_time)
    end

    print("done loop")

    -- tell R3 that R2 is in place
    sim.setIntegerSignal(channel2,0) --chat on CH2

    -- Wait for signal from R3 (rod in place)
    while (sim.getIntegerSignal(robot_names[3].."_CH2") ~= 1) do
        sim.wait(0.25)
    end

    actuateGripper("open") -- close gripper
    sim.wait(wait_time)

    -- return to starting position
    moveToConfig_viaFK(maxVel,maxAccel,maxJerk,startConfig,data)
end