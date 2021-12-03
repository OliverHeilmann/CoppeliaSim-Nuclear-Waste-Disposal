----------------------------------------------------------------------------------
-------------------------------- SETUP SECTION BELOW -----------------------------
----------------------------------------------------------------------------------
function sysCall_init()
    corout=coroutine.create(coroutineMain)  -- create main coroutine
end

function sysCall_actuation()
    if coroutine.status(corout)~='dead' then
        local ok,errorMsg=coroutine.resume(corout)
        if errorMsg then
            error(debug.traceback(corout,errorMsg),2)
        end
    end
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

----------------------------------------------------------------------------------
-------------------------------- MAIN SECTION BELOW ------------------------------
----------------------------------------------------------------------------------
function coroutineMain()
    -- Initialize some values:
    local simJoints={}
    for i=1,6,1 do
        simJoints[i]=sim.getObjectHandle('IRB140_joint'..i)
    end

    -- Show users which joints have been enabled
    print("Joints Handles: ", simJoints)

    local simTip=sim.getObjectHandle('IRB140_tip')
    local simTarget=sim.getObjectHandle('IRB140_manipulationSphere')
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
    -- MAIN LOOP
    while true do
        moveToConfig_dXYZ(ikMaxVel_small,ikMaxAccel_small,ikMaxJerk_small,data,simTarget,0,0,-0.12,0,0,0,0)
        sim.wait(1)

        moveToConfig_dXYZ(ikMaxVel_small,ikMaxAccel_small,ikMaxJerk_small,data,simTarget,0,0,0.06,0,0,0,0)
        sim.wait(1)

        moveToConfig_dXYZ(ikMaxVel_small,ikMaxAccel_small,ikMaxJerk_small,data,simTarget,0,0,0.1,0,0,0,0)
        sim.wait(1)

        -- move to next robot
        for movestep = 1, 3 do
            moveToConfig_viaFK(maxVel,maxAccel,maxJerk,macro_moves[movestep],data)
            sim.wait(1)
        end
    end 

    moveToConfig_viaFK(maxVel,maxAccel,maxJerk,initConf,data)
    sim.wait(5)
    --sim.stopSimulation()
end