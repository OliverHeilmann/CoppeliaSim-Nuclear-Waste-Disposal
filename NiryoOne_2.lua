-- Create popup window with buttons
function uiSetup()
    xml = '<ui title="R2 Robot Arm Control" closeable="false" resizeable="true" activate="false">' .. [[
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
    sim.setStringSignal("R2", sim.packTable({myname}))
    
    -- Print for debugging
    print("R2: " .. simSend)
    print("R2: " .. simRecieve)
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
    target = sim.getObjectHandle("nodeIK_obj#0") -- CHANGE FOR THE APPROPRIATE TARGET!
    
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
    -- MOVE TO R1/R2 LOCATION, THEN GRAB ROD FROM R1

    -- move away from wall
    local pos = {0.018514279276133, 0.53788828849792, -0.89212203025818, 0.0064570941030979, -1.2166624069214, -0.00010122731328011}
    moveToConfig(jointHandles,maxVel,maxAccel,maxJerk,pos)

    -- move to niryo 1
    local pos0 =  {-0.50411337614059, -0.61416971683502, 0.28874540328979, -1.6140213012695, -1.0551534891129, 0.30452990531921}
    moveToConfig(jointHandles,maxVel,maxAccel,maxJerk,pos0)

    -- Wait for signal from R1 (rod in place)
    while (sim.getIntegerSignal(robot_names[1].."_CH1") ~= 1) do
        sim.wait(0.25)
    end
    
    -- grab object with gripper
    sim.setIntegerSignal(gripperName.. '_close', 1) --close
    sim.wait(7)

    -- tell R1 that R2 has closed gripper
    sim.setIntegerSignal(simSend,1)
    sim.wait(3)


    ----------------------------------------------------------------
    -- R2 MOVES TO R3 TO TRANSFER ROD

    -- move away from wall safely
    local pos2 = {-0.82792115211487, -0.071400940418243, -0.43170803785324, -1.9132986068726, -0.79814648628235, 0.58873349428177}
    moveToConfig(jointHandles,maxVel,maxAccel,maxJerk,pos2)

    -- move to niryoOne 2
    local pos3 = {-1.6819672584534, -0.45044577121735, 0.055385649204254, -5.9355748817325e-05, -1.1760159730911, -1.2}
    moveToConfig(jointHandles,maxVel,maxAccel,maxJerk,pos3)

    local pos4 = {-2.055597782135, -0.49194490909576, 0.11541658639908, 8.5830688476562e-06, -1.1943854093552, 1.0860061645508}
    moveToConfig(jointHandles,maxVel,maxAccel,maxJerk,pos4)

    local pos5 = {-2.292445898056, -0.79580140113831, 0.58038860559464, -0.00023745000362396, -1.3571333885193, 0.8492077589035}
    moveToConfig(jointHandles,maxVel,maxAccel,maxJerk,pos5)

    -- tell R3 that R2 is in place
    sim.setIntegerSignal(simRecieve,1) --chat on CH2

    -- Wait for signal from R1 (rod in place)
    while (sim.getIntegerSignal(robot_names[3].."_CH2") ~= 1) do
        sim.wait(0.25)
    end

    sim.clearIntegerSignal(gripperName.. '_close') --open
    sim.wait(4)

end
