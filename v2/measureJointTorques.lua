function sysCall_init()
    -- init graphing function/ setup properties
    graph=sim.getObjectHandle('R1_Graph')

    -- create some colours for plotting joint torques
    colours = {{1,0,0}, {0,1,0}, {0,0,1},{0.5,0.5,0}, {0,0.5,0.5}, {1,1,1}}

    -- Get all the joint handles for R1
    simJoints={}
    distStreams={}
    for i=1,6,1 do
        simJoints[i]=sim.getObjectHandle('NiryoOneJoint'..i)
        local mystring = "Joint "..tostring(i).." Torque"
        distStreams[i]=sim.addGraphStream(graph,mystring,'N',0,colours[i])
    end

   
end

function sysCall_sensing()
    -- update joint torques
    for i=1,6,1 do
        forceOrTorque=sim.getJointForce(simJoints[i])
        sim.setGraphStreamValue(graph,distStreams[i],forceOrTorque)
    end
end