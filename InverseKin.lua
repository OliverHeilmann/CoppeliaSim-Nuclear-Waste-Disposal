----------------------------------------------------------------------------------
-------------------------------- POPUP SECTION BELOW -----------------------------
----------------------------------------------------------------------------------
-- Create popup window with buttons
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
    print("MAKE SURE TO SET THE IK SOLVER TO THE CORRECT ROBOT REFERENCE!!!!")
    sim.setStringSignal("R1", sim.packTable({myname}))
    
    -- Print for debugging
    print("IK: " .. simSend)
    print("IK: " .. simRecieve)
end

-- add x,y,z,r,p,y values for reference to set starting point for IK
function setReferencePos()
    local xyz =   {-7.2102120611817e-05, 0.34925019741058, 0.25622445344925}
    local rpy = {-1.5875544548035, -0.0001523432583781, 0.00034221456735395}

    -- Get handle of target to allow for changing state
    target = sim.getObjectHandle("referenceIK_obj"..robotnum) -- CHANGE FOR THE APPROPRIATE TARGET!

    -- change target state
    sim.setObjectPosition(target, -1, xyz)
    sim.setObjectOrientation(target, -1, rpy)
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
    target = sim.getObjectHandle("nodeIK_obj"..robotnum) -- CHANGE FOR THE APPROPRIATE TARGET!
    
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
-- Main thread
function sysCall_init()
    robotnum = "" --"#2" --define robot number
    -- ALSO CHANGE THE R1 TO THE NEW ONE!!!

    -- Setup UI interactive boxes
    uiSetup()

    -- Setup messaging between robots
    messageSetup()

    -- Set a starting location for reference dummy
    setReferencePos()

    -- Take a few handles from the scene:
    simBase=sim.getObjectHandle(sim.handle_self)
    simTip=sim.getObjectHandle('nodeIK_obj'..robotnum)
    simTarget=sim.getObjectHandle('referenceIK_obj'..robotnum)
    
    ikEnv=simIK.createEnvironment()
    
    -- Prepare the 2 ik groups, using the convenience function 'simIK.addIkElementFromScene':
    ikGroup_undamped=simIK.createIkGroup(ikEnv)
    simIK.setIkGroupCalculation(ikEnv,ikGroup_undamped,simIK.method_pseudo_inverse,0,10)
    simIK.addIkElementFromScene(ikEnv,ikGroup_undamped,simBase,simTip,simTarget,simIK.constraint_pose)
    ikGroup_damped=simIK.createIkGroup(ikEnv)
    simIK.setIkGroupCalculation(ikEnv,ikGroup_damped,simIK.method_damped_least_squares,0.3,99)
    simIK.addIkElementFromScene(ikEnv,ikGroup_damped,simBase,simTip,simTarget,simIK.constraint_pose)

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

-- Main thread process
function sysCall_actuation()
    if simIK.applyIkEnvironmentToScene(ikEnv,ikGroup_undamped,true)~=simIK.result_success then
        simIK.applyIkEnvironmentToScene(ikEnv,ikGroup_damped)
    end

end

function sysCall_cleanup()
    simIK.eraseEnvironment(ikEnv)
end
