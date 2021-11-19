-- Create popup window with buttons
function uiSetup()
    xml = '<ui title="R3 Robot Arm Control" closeable="false" resizeable="true" activate="false">' .. [[
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
    sim.setStringSignal("R3", sim.packTable({myname}))
    
    -- Print for debugging
    print("R3: " .. simSend)
    print("R3: " .. simRecieve)
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
    target = sim.getObjectHandle("nodeIK_obj#1") -- CHANGE FOR THE APPROPRIATE TARGET!
    
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
    -- MOVE TO R2/R3 LOCATION, THEN GRAB ROD FROM R2

    -- move away from wall
    local pos = {0.018514279276133, 0.53788828849792, -0.89212203025818, 0.0064570941030979, -1.2166624069214, -0.00010122731328011}
    moveToConfig(jointHandles,maxVel,maxAccel,maxJerk,pos)

    -- move ready to collect rod from R2
    local pos1 = {-0.44177311658859, -0.91892397403717, 0.18760776519775, -1.8766129016876, -1.2468864917755, 0.7809898853302}
    moveToConfig(jointHandles,maxVel,maxAccel,maxJerk,pos1)

    local pos2 = {-0.41322028636932, -1.0203392505646, 0.37898004055023, -1.8254935741425, -1.2424811124802, 0.68131577968597}
    moveToConfig(jointHandles,maxVel,maxAccel,maxJerk,pos2)

    -- Wait for signal from R2 (rod in place)
    while (sim.getIntegerSignal(robot_names[2].."_CH2") ~= 1) do
        sim.wait(0.25)
    end
    
    -- grab object with gripper
    sim.setIntegerSignal(gripperName.. '_close', 1) --close
    sim.wait(7)

    -- tell R1 that R2 has closed gripper
    sim.setIntegerSignal(simRecieve,1)
    sim.wait(3)

    -- move away from R2 to complete pass over
    local pos3 = {-0.83444774150848, -0.42017310857773, -0.7326043844223, -2.3592672348022, -1.2636750936508, 1.2788199186325}
    moveToConfig(jointHandles,maxVel,maxAccel,maxJerk,pos3)

    local pos4 = {-0.62785005569458, -0.38807138800621, -0.65722018480301, -2.3943223953247, -1.0484414100647, -0.37324190139771}
    moveToConfig(jointHandles,maxVel,maxAccel,maxJerk,pos4)

    local pos5 = {1.2516194581985, -0.22126001119614, -0.26860845088959, -3.0543649196625, 1.0777109861374, 1.209512591362}
    moveToConfig(jointHandles,maxVel,maxAccel,maxJerk,pos5)

    local pos6 = {1.9784574508667, -0.76329916715622, 0.27675852179527, 0.072958767414093, -1.1067006587982, -1.1951094865799}
    moveToConfig(jointHandles,maxVel,maxAccel,maxJerk,pos6)

    local pos7 = {2.2673468589783, -1.2884149551392, 1.2268596887589, 0.068678915500641, -1.5113272666931, -0.87699472904205}
    moveToConfig(jointHandles,maxVel,maxAccel,maxJerk,pos7)

    -- tell R4 that R3 is in place
    sim.setIntegerSignal(simSend,1) --chat on CH1

    -- Wait for signal from R4 (rod in place)
    while (sim.getIntegerSignal(robot_names[4].."_CH1") ~= 1) do
        sim.wait(0.25)
    end

    sim.clearIntegerSignal(gripperName.. '_close') --open
    sim.wait(4)

end
