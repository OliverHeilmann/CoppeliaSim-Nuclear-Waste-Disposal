----------------------------------------------------------------------------------
-------------------------------- POPUP SECTION BELOW -----------------------------
----------------------------------------------------------------------------------
function uiSetup()
    xml = '<ui title="R1 Robot Arm Control" closeable="false" resizeable="true" activate="false">' .. [[
                    <button text="Print Joint Angles" on-click="printJointAngles" id="1"/>
                    <button text="Print Target State" on-click="printTargetPosition" id="2"/>
                    <button text="Open Gripper" on-click="actuateGripperUI" id="1001" />
                    <button text="Close Gripper" on-click="actuateGripperUI" id="1002" />
                    <button text="Print Vision" on-click="printVisionUI" id="3" />
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
    target = sim.getObjectHandle("nodeIK_obj") -- CHANGE FOR THE APPROPRIATE TARGET!
    
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

function printVisionUI()
    -- Get returned information from vision sensor
    local handle = sim.getObjectHandle ('gripperVisionSensor')
    if (sim.isHandleValid (handle)) then
        _, _, unpackedPacket1 = sim.handleVisionSensor (handle)
        _, _, unpackedPacket2 = sim.readVisionSensor(handle)

        -- now print readable data to console
        print(unpackedPacket2)
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
    sim.setStringSignal("R1", sim.packTable({myname}))
    
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

function moveToConfig_dXYZ(ikMaxVel,ikMaxAccel,ikMaxJerk,data,mytarget,dX,dY,dZ)
    -- get target handle
    local pose = sim.getObjectPose(mytarget,-1)

    -- adjust current pose to user defined pose
    pose[1] = pose[1] + dX  -- global x
    pose[2] = pose[2] + dY  -- global y
    pose[3] = pose[3] + dZ  -- global z

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
    local simTip=sim.getObjectHandle('nodeIK_obj')
    local simTarget=sim.getObjectHandle('referenceIK_obj')
    local modelBase=sim.getObjectHandle(sim.handle_self)
    
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

    local pickConfig={0.0021278122439981, -0.58866262435913, 0.58699184656143, 7.349462248385e-05, -1.5762243270874, 0.0019820025190711}
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
    sim.waitForSignal("R2") --("R4") -- wait for final robot to init
    robot_names = {}
    sig_names = {"R1", "R2"} --, "R3", "R4"}
    for i=1, 2,1 do --4, 1 do
        local theirName = sim.unpackTable( sim.getStringSignal(sig_names[i]) )
        robot_names[i] = theirName[1]
    end

    ----------------------------------------------------------------
    -- MAIN LOOP
    fuelRods = {"fuelCentre", "fuelCentre#0", "fuelCentre#1", "fuelCentre#2"}
    wait_time = 4 -- time to wait between movements (seconds)
    for i = 1, 4 do
        moveToConfig_viaFK(maxVel,maxAccel,maxJerk,pickConfig,data)
        sim.wait(wait_time)

        --[[
        -- Get returned information from vision sensor
        Hand = sim.getObjectHandle ('gripperVisionSensor')
        if (sim.isHandleValid (Hand)) then
            while (true) do
                _, _, handle = sim.handleVisionSensor (Hand)
                _, _, Data = sim.readVisionSensor(Hand)
                print(Data)
            end
        end
        --]]

        local fueltip=sim.getObjectHandle(fuelRods[i])
        local poseTarget=sim.getObjectPose(fueltip,-1)
        poseTarget[3]=poseTarget[3]+0.12
        moveToPose_viaIK(ikMaxVel,ikMaxAccel,ikMaxJerk,poseTarget,data)
        sim.wait(wait_time)

        moveToConfig_dXYZ(ikMaxVel_small,ikMaxAccel_small,ikMaxJerk_small,data,simTarget, 0, 0, -0.07)
        sim.wait(wait_time)

        actuateGripper("close") -- close gripper

        moveToConfig_dXYZ(ikMaxVel_small,ikMaxAccel_small,ikMaxJerk_small,data,simTarget, 0, 0, 0.06)
        sim.wait(3)
        moveToConfig_dXYZ(ikMaxVel_small,ikMaxAccel_small,ikMaxJerk_small,data,simTarget, 0, 0, 0.1)
        sim.wait(3)

        -- move to next robot
        for movestep = 1, 3 do
            moveToConfig_viaFK(maxVel,maxAccel,maxJerk,macro_moves[movestep],data)
            sim.wait(wait_time)
        end

        -- tell R2 that R1 is in place
        sim.setIntegerSignal(channel1,1)

        ----------------------------------------------------------------
        -- WAITING FOR R2 TO GRAB ROD FROM R1, LETS GO WHEN R2 GRABBED
        while (sim.getIntegerSignal(robot_names[2].."_CH1") ~= 1) do
            sim.wait(0.25)
        end

        actuateGripper("open") -- open gripper
        sim.wait(wait_time)

        -- tell R1 no longer in place, set channel 1 back to 0
        sim.setIntegerSignal(channel1,0)
    end

    moveToConfig_viaFK(maxVel,maxAccel,maxJerk,initConf,data)
    sim.stopSimulation()
    
end