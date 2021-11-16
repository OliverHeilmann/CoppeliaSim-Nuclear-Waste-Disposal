-- Create popup window with buttons
function uiSetup()
    xml = '<ui title="Robot Arm Control" closeable="false" resizeable="true" activate="false">' .. [[
                    <button text="Print Joint Angles" on-click="printJointAngles" id="1"/>
                    <radiobutton text="Open Gripper" onclick="gripperOpen" id="1001" />
                    <radiobutton text="Close Gripper" onclick="gripperClose" id="1002" />
                    <label text="" style="* {margin-left: 200px;}"/>        
                    </ui>
                    ]]

    uii=simUI.create(xml)
end

function sysCall_init()
    corout=coroutine.create(coroutineMain)

    -- call UI box setup
    uiSetup()
end

function sysCall_actuation()
    if coroutine.status(corout)~='dead' then
        local ok,errorMsg=coroutine.resume(corout)
        if errorMsg then
            error(debug.traceback(corout,errorMsg),2)
        end
    end
end

-- This is a threaded script, and is just an example!

--[[
function radiobuttonClick(ui,id)
    local sel={'realtosim','simtoreal','C'}
    simUI.setLabelText(ui,1000,'You selected '..sel[id-1000])
    status = id - 1000
end
--]]

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

function movCallback(config,vel,accel,handles)
    for i=1,#handles,1 do
        if sim.getJointMode(handles[i])==sim.jointmode_force and sim.isDynamicallyEnabled(handles[i]) then
            sim.setJointTargetPosition(handles[i],config[i])
        else    
            sim.setJointPosition(handles[i],config[i])
        end
    end
end

function moveToConfig(handles,maxVel,maxAccel,maxJerk,targetConf)
    local currentConf={}
    for i=1,#handles,1 do
        currentConf[i]=sim.getJointPosition(handles[i])
    end
    sim.moveToConfig(-1,currentConf,nil,nil,maxVel,maxAccel,maxJerk,targetConf,nil,movCallback,handles)
end

function coroutineMain()
    local jointHandles={}
    for i=1,6,1 do
        jointHandles[i]=sim.getObjectHandle('NiryoOneJoint'..i)
    end

    -- Print the current joint angles
    local currentConf={}
    for i=1,#jointHandles,1 do
        currentConf[i]=sim.getJointPosition(jointHandles[i])
    end
    print(currentConf)

    local connection=sim.getObjectHandle('NiryoOne_connection')
    local gripper=sim.getObjectChild(connection,0)
    local gripperName="NiryoNoGripper"
    if gripper~=-1 then
        gripperName=sim.getObjectName(gripper)
    end
    
    -- Set-up some of the RML vectors:
    local vel=20
    local accel=40
    local jerk=80
    local maxVel={vel*math.pi/180,vel*math.pi/180,vel*math.pi/180,vel*math.pi/180,vel*math.pi/180,vel*math.pi/180}
    local maxAccel={accel*math.pi/180,accel*math.pi/180,accel*math.pi/180,accel*math.pi/180,accel*math.pi/180,accel*math.pi/180}
    local maxJerk={jerk*math.pi/180,jerk*math.pi/180,jerk*math.pi/180,jerk*math.pi/180,jerk*math.pi/180,jerk*math.pi/180}

    
    local targetPos1={0,0,-0.5, 0,0,0}
    moveToConfig(jointHandles,maxVel,maxAccel,maxJerk,targetPos1)
    sim.setIntegerSignal(gripperName..'_open',1)
    sim.wait(5)


    local targetPos1={90*math.pi/180,-54*math.pi/180,0*math.pi/180,0*math.pi/180,-36*math.pi/180,-90*math.pi/180}
    moveToConfig(jointHandles,maxVel,maxAccel,maxJerk,targetPos1)
    sim.setIntegerSignal(gripperName..'_close',1)
    sim.wait(5)

    local targetPos3={0,0,0,0,0,0}
    moveToConfig(jointHandles,maxVel,maxAccel,maxJerk,targetPos3)

    local targetPos2={-90*math.pi/180,-54*math.pi/180,0*math.pi/180,0*math.pi/180,-36*math.pi/180,-90*math.pi/180}
    moveToConfig(jointHandles,maxVel,maxAccel,maxJerk,targetPos2)
    sim.clearIntegerSignal(gripperName..'_open')
    sim.wait(4)

    moveToConfig(jointHandles,maxVel,maxAccel,maxJerk,targetPos3)
    

end
