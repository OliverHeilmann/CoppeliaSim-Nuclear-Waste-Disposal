-- Create popup window with buttons
function uiSetup()
    xml = '<ui title="R1 Robot Arm Control" closeable="false" resizeable="true" activate="false">' .. [[
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
    sim.setStringSignal("R1", sim.packTable({myname}))
    
    -- Print for debugging
    print("R1: " .. simSend)
    print("R1: " .. simRecieve)
end

-- Initialize
function sysCall_init()
    corout=coroutine.create(coroutineMain)

    -- call UI box setup
    uiSetup()

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
    target = sim.getObjectHandle("nodeIK_obj") -- CHANGE FOR THE APPROPRIATE TARGET!
    
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

-- Print Target position in console window
function get_obj_state(target_name)
    -- collect coordinates and pose
    local target = sim.getObjectHandle(target_name)
    local pos = sim.getObjectPosition(target,-1)
    local orientation = sim.getObjectOrientation(target,-1)

    -- print to console window
    return pos, orientation
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
    -- R1 TO THE ROD
    
    -- move to rod (macro movement)
    local pos1 = {0.0023124739527702, -0.42409363389015, 0.018077529966831, -4.0032842662185e-05, -1.16523873806, 0.0023059388622642}
    moveToConfig(jointHandles,maxVel,maxAccel,maxJerk,pos1)
    
    -- move to rod (micro movement)
    local pos2 = {0.0023129507899284, -0.50707757472992, -0.092169612646103, -3.5741308238357e-05, -0.97195827960968, 0.0023080846294761}
    moveToConfig(jointHandles,maxVel,maxAccel,maxJerk,pos2)
    
    -- grab rod with gripper
    sim.setIntegerSignal(gripperName.. '_close', 1) --close
    sim.wait(5)

    -- move to rod (macro movement)
    local pos3 = {0.0023124739527702, -0.42409363389015, 0.018077529966831, -4.0032842662185e-05, -1.16523873806, 0.0023059388622642}
    moveToConfig(jointHandles,maxVel,maxAccel,maxJerk,pos3)
    
    -- move away from rod matrix
    local pos4 = {0.0022422843612731, -0.4016324877739, 0.7152601480484, 0.00019060660270043, -1.7453393936157, 0.0022931143175811}
    moveToConfig(jointHandles,maxVel,maxAccel,maxJerk,pos4)

    -- move to niryoOne R2
    local pos5 = {-1.7891311645508, -0.41722574830055, -0.032950282096863, -1.6671049594879, -1.3744693994522, -1.1116399765015}
    moveToConfig(jointHandles,maxVel,maxAccel,maxJerk,pos5)

    -- move to niryoOne R2
    local pos6 = {-2.0959885120392, -0.59904563426971, 0.23584520816803, -1.7741186618805, -1.0830663442612, -1.1563498973846}
    moveToConfig(jointHandles,maxVel,maxAccel,maxJerk,pos6)
    
    -- tell R2 that R1 is in place
    sim.setIntegerSignal(simSend,1)


    ----------------------------------------------------------------
    -- WAITING FOR R2 TO GRAB ROD FROM R1, LETS GO WHEN R2 GRABBED

    -- wait for signal from R2 (wait for R2 to close gripper)
    while (sim.getIntegerSignal(robot_names[2].."_CH1") ~= 1) do
        sim.wait(0.25)
    end

    sim.clearIntegerSignal(gripperName.. '_close') --open
    sim.wait(4)


    ----------------------------------------------------------------
    -- R1 IS NOW SAFE TO MOVE BACK TO STARTING AREA FOR NEXT ROD

    -- move back to start, high above the rod matrix
    moveToConfig(jointHandles,maxVel,maxAccel,maxJerk,pos4)
end
