function sysCall_init()
    -- init graphing function/ setup properties
    graph=sim.getObjectHandle('R1_Graph#0')

    -- create some colours for plotting joint torques
    colours = {{1,0,0}, {0,1,0}, {0,0,1},{0.5,0.5,0}, {0,0.5,0.5}, {1,1,1}}

    -- Get all the joint handles for R1
    simJoints={}
    distStreams={}

    -- choose from 1 to n joints. Selected will be plotted to R1_graph (1,4,6), (2,3,5)
    jointsTarget = {1,4,6}

    for i=1,3,1 do
        simJoints[i]=sim.getObjectHandle('NiryoOneJoint'..jointsTarget[i])
        local mystring = "Joint "..tostring(jointsTarget[i]).." Torque"
        distStreams[i]=sim.addGraphStream(graph,mystring,'N',0,colours[jointsTarget[i]])
    end
   
end

function sysCall_sensing()
    -- update joint torques
    for i=1,3,1 do
        forceOrTorque=sim.getJointForce(simJoints[i])
        sim.setGraphStreamValue(graph,distStreams[i],forceOrTorque)
    end
end