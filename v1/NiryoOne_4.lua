-- Create popup window with buttons
function uiSetup()
    xml = '<ui title="R4 Robot Arm Control" closeable="false" resizeable="true" activate="false">' .. [[
                    <button text="Print Joint Angles" on-click="printJointAngles" id="1"/>
                    <button text="Print Target State" on-click="printTargetPosition" id="2"/>
                    <button text="Open Gripper" on-click="actuateGripper" id="1001" />
                    <button text="Close Gripper" on-click="actuateGripper" id="1002" />
                    <label text="" style="* {margin-left: 200px;}"/>        
                    </ui>
                    ]]

    uii=simUI.create(xml)
end

-- Messaging between Robots setup
function messageSetup()
    -- Store global name of me
    myhandle = sim.getObjectHandle(sim.handle_self)
    myname = sim.getObjectName(myhandle)

    -- Setup sending and recieving globally accessible messages
    simSend = myname.."_CH1"
    simRecieve = myname .."_CH2"

    -- Send messages on channels
    sim.setIntegerSignal(simSend,0)
    sim.setIntegerSignal(simRecieve,0)

    -- Publish my name so others can access it
    sim.setStringSignal("R4", sim.packTable({myname}))
    
    -- Print for debugging
    print("R4: " .. simSend)
    print("R4: " .. simRecieve)
end


-- Initialize
function sysCall_init()
    corout=coroutine.create(coroutineMain)

    -- call UI box setup
    --uiSetup()

    -- call messaging setup
    messageSetup()

    -- Get joint handles and store as global variable
    jointHandles = {}
    for i=1,6,1 do
        jointHandles[i]=sim.getObjectHandle('NiryoOneJoint'..i)
    end

    -- Get gripper name and set as global variable
    local connection=sim.getObjectHandle('NiryoOne_connection')
    local gripper=sim.getObjectChild(connection,0)
    gripperName = "NiryoNoGripper"
    if gripper~=-1 then
        gripperName=sim.getObjectName(gripper)
    end
end

-- Check if CoppeliaSim connection has been made
function sysCall_actuation()
    if coroutine.status(corout)~='dead' then
        local ok,errorMsg=coroutine.resume(corout)
        if errorMsg then
            error(debug.traceback(corout,errorMsg),2)
        end
    end
end

-- Print joint angles into console window
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

-- Print Target position in console window
function printTargetPosition()
    target = sim.getObjectHandle("nodeIK_obj#2") -- CHANGE FOR THE APPROPRIATE TARGET!
    
    -- get position and orientation of target
    pos = sim.getObjectPosition(target,-1)
    orientation = sim.getObjectOrientation(target,-1)

    -- print to console window
    print(pos, orientation)
end

-- Open/ Close Gripper
function actuateGripper(ui, id)
    if id == 1001 then
        -- open gripper
        sim.clearIntegerSignal(gripperName.. '_close')
    else
        -- close gripper
        sim.setIntegerSignal(gripperName.. '_close', 1)
    end
end

-- move Niryo to a define state based on input joint angles
function movCallback(config,vel,accel,handles)
    for i=1,#handles,1 do
        if sim.getJointMode(handles[i])==sim.jointmode_force and sim.isDynamicallyEnabled(handles[i]) then
            sim.setJointTargetPosition(handles[i],config[i])
        else    
            sim.setJointPosition(handles[i],config[i])
        end
    end
end

-- Pass joint names to moveCallback function
function moveToConfig(handles,maxVel,maxAccel,maxJerk,targetConf)
    local currentConf={}
    for i=1,#handles,1 do
        currentConf[i]=sim.getJointPosition(handles[i])
    end
    sim.moveToConfig(-1,currentConf,nil,nil,maxVel,maxAccel,maxJerk,targetConf,nil,movCallback,handles)
end

-- Main threaded function
function coroutineMain()
    ----------------------------------------------------------------
    -- GET ALL OTHER ROBOT NAMES IN CHAIN

    -- ensure final robot has finished making their reference name
    sim.waitForSignal("R4")

    -- get all robot names dynamically
    robot_names = {}
    sig_names = {"R1", "R2", "R3", "R4"}
    for i=1, 4, 1 do

        local theirName = sim.unpackTable( sim.getStringSignal(sig_names[i]) )
        robot_names[i] = theirName[1]
    end

    ----------------------------------------------------------------
    -- SETUP SOME OF THE RML VECTORS
    local vel=20
    local accel=40
    local jerk=80
    local maxVel={vel*math.pi/180,vel*math.pi/180,vel*math.pi/180,vel*math.pi/180,vel*math.pi/180,vel*math.pi/180}
    local maxAccel={accel*math.pi/180,accel*math.pi/180,accel*math.pi/180,accel*math.pi/180,accel*math.pi/180,accel*math.pi/180}
    local maxJerk={jerk*math.pi/180,jerk*math.pi/180,jerk*math.pi/180,jerk*math.pi/180,jerk*math.pi/180,jerk*math.pi/180}


    ----------------------------------------------------------------
    -- MOVE TO R3/R4 LOCATION, THEN GRAB ROD FROM R3

    -- move away from wall
    local pos = {0.018514279276133, 0.53788828849792, -0.89212203025818, 0.0064570941030979, -1.2166624069214, -0.00010122731328011}
    moveToConfig(jointHandles,maxVel,maxAccel,maxJerk,pos)

    -- move to niryo 1
    local pos1 =  {0.61427265405655, -1.1134645938873, 0.34770035743713, 2.0251832008362, -1.1422690153122, -0.86586511135101}
    moveToConfig(jointHandles,maxVel,maxAccel,maxJerk,pos1)

    -- Wait for signal from R1 (rod in place)
    while (sim.getIntegerSignal(robot_names[3].."_CH1") ~= 1) do
        sim.wait(0.25)
    end
    
    -- grab object with gripper
    sim.setIntegerSignal(gripperName.. '_close', 1) --close
    sim.wait(7)

    -- tell R1 that R2 has closed gripper
    sim.setIntegerSignal(simSend,1)
    sim.wait(3)

    local pos3 = {1.5256195068359, -0.64317941665649, -0.66680145263672, 3.0542986392975, -1.3046585321426, -1.5476306676865}
    moveToConfig(jointHandles,maxVel,maxAccel,maxJerk,pos3)

    local pos4 =  {3.054322719574, 0.086541354656219, -0.23199617862701, 1.8568181991577, -0.14869892597198, -1.925687789917}
    moveToConfig(jointHandles,maxVel,maxAccel,maxJerk,pos4)

    --local pos5 = {-1.69868683815, -0.64544856548309, 1.0028569698334, -0.099651575088501, -1.7453370094299, -0.16341280937195}
    --moveToConfig(jointHandles,maxVel,maxAccel,maxJerk,pos5)


    sim.clearIntegerSignal(gripperName.. '_close') --open
    sim.wait(4)

end