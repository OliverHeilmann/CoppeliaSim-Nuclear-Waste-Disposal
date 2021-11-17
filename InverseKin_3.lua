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

function sysCall_init()
    -- Setup UI interactive boxes
    uiSetup()
    
    -- Take a few handles from the scene:
    simBase=sim.getObjectHandle(sim.handle_self)
    simTip=sim.getObjectHandle('nodeIK_obj')
    simTarget=sim.getObjectHandle('referenceIK_obj')
    
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


-- Print joint angles into console window
function printJointAngles()
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


function sysCall_actuation()

    if simIK.applyIkEnvironmentToScene(ikEnv,ikGroup_undamped,true)~=simIK.result_success then
        simIK.applyIkEnvironmentToScene(ikEnv,ikGroup_damped)
    end
end

function sysCall_cleanup()
    simIK.eraseEnvironment(ikEnv)
end
