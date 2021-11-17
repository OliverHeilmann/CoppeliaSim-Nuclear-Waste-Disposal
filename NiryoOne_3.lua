-- Create popup window with buttons
function uiSetup()
    xml = '<ui title="Robot Arm Control" closeable="false" resizeable="true" activate="false">' .. [[
                    <button text="Print Joint Angles" on-click="printJointAngles" id="1"/>
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
    myname = sim.handle_self

    -- Setup sending and recieving globally accessible messages
    simSend = tostring(sim.getObjectHandle(myname)).."_Send"
    simRecieve = tostring(sim.getObjectHandle(myname)) .. "_Recieve"

    -- Send messages on channels
    sim.setIntegerSignal(simSend,0)
    sim.setIntegerSignal(simRecieve,0)
    
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

    -- Set-up some of the RML vectors:
    local vel=20
    local accel=40
    local jerk=80
    local maxVel={vel*math.pi/180,vel*math.pi/180,vel*math.pi/180,vel*math.pi/180,vel*math.pi/180,vel*math.pi/180}
    local maxAccel={accel*math.pi/180,accel*math.pi/180,accel*math.pi/180,accel*math.pi/180,accel*math.pi/180,accel*math.pi/180}
    local maxJerk={jerk*math.pi/180,jerk*math.pi/180,jerk*math.pi/180,jerk*math.pi/180,jerk*math.pi/180,jerk*math.pi/180}

    -- ensure upchain robot has initiated its messaging service
    sim.waitForSignal("97_Send")


    ----------------------------------------------------------------
    -- MOVE TO R2/R3 LOCATION, THEN GRAB ROD FROM R2

    -- move ready to collect rod from R2
    local pos1 =  {-0.81381344795227, -1.1193515062332, 0.6615344285965, -1.1757044792175, 0.90699589252472, 0.97616267204285}
    moveToConfig(jointHandles,maxVel,maxAccel,maxJerk,pos1)

    local pos2 = {-1.522524356842, 0.19659888744354, -0.79597449302673, 0.085453987121582, 0.60091388225555, -0.070513248443604}
    moveToConfig(jointHandles,maxVel,maxAccel,maxJerk,pos2)

end
