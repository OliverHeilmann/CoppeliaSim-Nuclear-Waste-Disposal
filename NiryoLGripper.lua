function sysCall_init()
    gripperName=sim.getObjectName(sim.getObjectHandle(sim.handle_self))
    leftSensor=sim.getObjectHandle('NiryoLGripper_leftSensor')
    rightSensor=sim.getObjectHandle('NiryoLGripper_rightSensor')
    attachPtA=sim.getObjectHandle('NiryoLGripper_attachPtA')
    attachPtB=sim.getObjectHandle('NiryoLGripper_attachPtB')
    motor=sim.getObjectHandle('NiryoLGripper_rightJoint1')
    maxV=10*math.pi/180 -- change vals here for velocity!
    closed=false
    delay=-1
end

function sysCall_actuation()
    local closing=sim.getIntegerSignal(gripperName..'_close')
    if closing then
        if not closed then
            -- Close:
            sim.setJointTargetVelocity(motor,-maxV)
            local lresult,ldist,lPt,lHandle=sim.handleProximitySensor(leftSensor)
            local rresult,rdist,rPt,rHandle=sim.handleProximitySensor(rightSensor)
            if lHandle and lHandle==rHandle then
                local iter=lHandle
                while iter~=-1 do
                    local isShape=sim.getObjectType(iter)==sim.object_shape_type
                    local dyn=false
                    if isShape then
                        local param=sim.getObjectInt32Param(iter,sim.shapeintparam_static)
                        dyn=(param==0)
                    end
                    if isShape and dyn then
                        if delay<0 then
                            delay=0.6
                        else
                            delay=delay-sim.getSimulationTimeStep()
                            if delay<0 then
                                sim.setJointTargetVelocity(motor,0)
                                sim.setObjectParent(attachPtB,iter,true)
                                closed=true
                            end
                        end
                        break
                    else
                        iter=sim.getObjectParent(iter)
                    end
                end
            end
        end
    else
        if closed then
            -- Detach a potentially attached object:
            sim.setObjectParent(attachPtB,attachPtA,true)
            sim.setObjectPosition(attachPtB,attachPtA,{0,0,0})
            sim.setObjectOrientation(attachPtB,attachPtA,{0,0,0})
            closed=false
        end
        -- Open:
        sim.setJointTargetVelocity(motor,maxV)
    end
end
