function sysCall_init()
    corout=coroutine.create(coroutineMain)
end

function sysCall_actuation()
    if coroutine.status(corout)~='dead' then
        local ok,errorMsg=coroutine.resume(corout)
        if errorMsg then
            error(debug.traceback(corout,errorMsg),2)
        end
    end
end

setGripperData=function(open,velocity,force)
    if not velocity then
        velocity=0.11
    end
    if not force then
        force=20
    end
    if not open then
        velocity=-velocity
    end
    
    local dat={}
    dat.velocity=velocity
    dat.force=force
    sim.writeCustomDataBlock(gripperHandle,'activity',sim.packTable(dat))
end

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

function coroutineMain()
    -- Initialize some values:
    local simJoints={}
    for i=1,6,1 do
        simJoints[i]=sim.getObjectHandle('NiryoOneJoint'..i)
    end
    local simTip=sim.getObjectHandle('nodeIK_obj#0')
    local simTarget=sim.getObjectHandle('referenceIK_obj#0')
    local modelBase=sim.getObjectHandle(sim.handle_self)
    --gripperHandle=sim.getObjectHandle('RG2')
    
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

    local pickConfig={-70.1*math.pi/180,18.85*math.pi/180,93.18*math.pi/180,68.02*math.pi/180,109.9*math.pi/180,90*math.pi/180}
    local dropConfig1={-183.34*math.pi/180,14.76*math.pi/180,78.26*math.pi/180,-2.98*math.pi/180,-90.02*math.pi/180,86.63*math.pi/180}
    local dropConfig2={-197.6*math.pi/180,14.76*math.pi/180,78.26*math.pi/180,-2.98*math.pi/180,-90.02*math.pi/180,72.38*math.pi/180}
    local dropConfig3={-192.1*math.pi/180,3.76*math.pi/180,91.16*math.pi/180,-4.9*math.pi/180,-90.02*math.pi/180,-12.13*math.pi/180}
    local dropConfig4={-189.38*math.pi/180,24.94*math.pi/180,64.36*math.pi/180,0.75*math.pi/180,-90.02*math.pi/180,-9.41*math.pi/180}

    local dropConfigs={dropConfig1,dropConfig2,dropConfig3,dropConfig4}
    local dropConfigIndex=1
    local droppedPartsCnt=0

    --setGripperData(true)
    --sim.setInt32Param(sim.intparam_current_page,0)

    local data={}
    data.ikEnv=ikEnv
    data.ikGroup=ikGroup
    data.tip=simTip
    data.target=simTarget
    data.joints=simJoints

    --moveToConfig_viaFK(maxVel,maxAccel,maxJerk,pickConfig,data)
    --sim.wait(10)

    while (true) do

        local pos1 = {0.0023124739527702, -0.42409363389015, 0.018077529966831, -4.0032842662185e-05, -1.16523873806, 0.0023059388622642}
        moveToConfig_viaFK(maxVel,maxAccel,maxJerk,pos1,data)

        sim.wait(10)


        local fueltip=sim.getObjectHandle("fuelCentre#0")
        local pose=sim.getObjectPose(fueltip,-1)
        --pose[1]=pose[1]+0.105
        moveToPose_viaIK(ikMaxVel,ikMaxAccel,ikMaxJerk,pose,data)

        sim.wait(10)
    end
    
    --[[
    while droppedPartsCnt<6 do
        moveToConfig_viaFK(maxVel,maxAccel,maxJerk,pickConfig,data)
        --sim.setInt32Param(sim.intparam_current_page,1)

        local pose=sim.getObjectPose(simTip,-1)
        pose[1]=pose[1]+0.105
        moveToPose_viaIK(ikMaxVel,ikMaxAccel,ikMaxJerk,pose,data)

        --setGripperData(false)
        sim.wait(0.5)

        pose[2]=pose[2]-0.2
        pose[3]=pose[3]+0.2
        moveToPose_viaIK(ikMaxVel,ikMaxAccel,ikMaxJerk,pose,data)

        --sim.setInt32Param(sim.intparam_current_page,0)

        moveToConfig_viaFK(maxVel,maxAccel,maxJerk,dropConfigs[dropConfigIndex],data)

        --sim.setInt32Param(sim.intparam_current_page,2)
        local pose=sim.getObjectPose(simTip,-1)
        local pose2=sim.copyTable(pose)
        pose[3]=0.025+0.05*math.floor(0.1+droppedPartsCnt/2)
        moveToPose_viaIK(ikMaxVel,ikMaxAccel,ikMaxJerk,pose,data)

        --setGripperData(true)
        sim.wait(0.5)

        moveToPose_viaIK(ikMaxVel,ikMaxAccel,ikMaxJerk,pose2,data)


        --sim.setInt32Param(sim.intparam_current_page,0)

        dropConfigIndex=dropConfigIndex+1
        if dropConfigIndex>4 then
            dropConfigIndex=1
        end

        droppedPartsCnt=droppedPartsCnt+1
    end
    --]]

    moveToConfig_viaFK(maxVel,maxAccel,maxJerk,initConf,data)
    sim.stopSimulation()
end