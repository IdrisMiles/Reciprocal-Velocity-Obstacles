#version 440 core

layout (local_size_x = 1) in;

uniform samplerBuffer neighbours;       //texture location 0
uniform samplerBuffer neighbour_ids;    //texture location 1
uniform samplerBuffer des_vel;          //texture location 2


// SSBO - destination for output
layout (std430, binding = 0) buffer New_Vel_Buffer
{
    vec4 new_vel[];
};

//----------------------------------------------------------------------------

void createSampleVelocities(in vec3 desVel, inout vec3 testVels[72])
{
    float speed = length(desVel);
    vec3 u_DesVel = desVel;
    normalize(u_DesVel);
    float theta = atan(u_DesVel.z,u_DesVel.x);

    for(int i=0;i<36;i+=2)
    {
        //testVels[i].xz    = desVel.xz;
        //testVels[i].xz    = desVel.xz;
        testVels[i].xz    = speed * vec2(cos(theta+radians(i*5)),sin(theta+radians(i*5)));
        testVels[i+1].xz  = speed * vec2(cos(theta-radians(i*5)),sin(theta-radians(i*5)));
    }
}

//----------------------------------------------------------------------------

bool pointLeftOfEdge(in vec3 point, in vec3 edge, in vec3 edgeOrig)
{
    vec2 tmp = vec2((point - edgeOrig).xz);
    mat2 det;
    det[0] = edge.xz;
    det[1] = tmp;
    float result = determinant(det);

    return (result >= 0);

}

//----------------------------------------------------------------------------

float timeToIntersection(in vec3 vel, in vec3 p1, in vec3 edge, in vec3 p2, in float time)
{
    mat2 edges;
    edges[0] = vel.xz;
    edges[1] = edge.xz;
    float detVE = determinant(edges);

    vec3 dist;
    mat2 edgeDist;
    float detED;
    float t;

    if(detVE != 0)
    {
        //edges will collide

        dist = p2 - p1;
        edgeDist[0] = dist.xz;
        edgeDist[1] = edge.xz;
        detED = determinant(edgeDist);
        t = - (detED / detVE);

        if(t>=0 && t<=time)
        {
            return t;
        }

    }

    return -1.0f;
}

//----------------------------------------------------------------------------

bool testVO(in vec3 testVel, in vec3 neighPos, in vec3 neighVel, in float neighRad,
            in vec3 agentPos, in vec3 agentVel, in float agentRad, out float tValuesNeigh[10], in int invoc)
{
    vec3 dist = neighPos - agentPos;
    vec3 perpDist;
    perpDist.x = dist.z;
    perpDist.y = -dist.x;
    normalize(perpDist);

    //combine radius
    float rad2 = agentRad + neighRad;

    //VO appex offset
    vec3 apexOffset = 0.5 * (agentVel + neighVel);

    // 3 points of initial VO
    vec3 p1 = agentPos + apexOffset;
    vec3 p2 = neighPos + (rad2 * perpDist) + apexOffset;
    vec3 p3 = neighPos - (rad2 * perpDist) + apexOffset;

    // 3 edges of initial VO
    vec3 E1 = p3 - p2;  //back of VO
    vec3 E2 = p1 - p3;  //side 1 of VO
    vec3 E3 = p2 - p1;  //side 2 of VO

    // Hypot and Adjacent of VO and new Hypot of new VO
    float A = length(dist);
    float H = length(E2);
    float newH = (A - rad2) * tan(asin(rad2 / H));

    // new edges
    normalize(E2);
    E2 *= newH;
    E3 *= newH;

    // new points of new VO
    vec3 p4 = p1 + E3;
    vec3 p5 = p1 - E2;

    // esges of new VO
    vec3 leftEdge = p2 - p4;    //left side of VO
    vec3 rightEdge = p3 - p5;   //right side of VO
    vec3 frontEdge = p5 - p4;   //front side of VO

    // check whether velocity results in agent being outside of VO
    bool leftPlane = pointLeftOfEdge(agentPos+testVel,leftEdge,p4);
    bool rightPlane = pointLeftOfEdge(agentPos+testVel,rightEdge,p5);
    bool frontPlane = pointLeftOfEdge(agentPos+testVel,frontEdge,p4);

    if(leftPlane || rightPlane || frontPlane)
    {
        // velocity outside of VO - acceptable
        return true;
    }

    // velocity inside VO, find time to collision
    float time = 1000000;
    float t1 = timeToIntersection(testVel,agentPos,frontEdge,p4,time);
    float t2 = timeToIntersection(testVel,agentPos,rightEdge,p5,time);
    float t3 = timeToIntersection(testVel,agentPos,leftEdge,p4,time);

    float MAX = 10000000;
    float tmp1 = (t1>0)?t1:MAX;
    float tmp2 = (t2>0)?t2:MAX;
    float tmp3 = (t3>0)?t3:MAX;

    float lowestT   = (tmp1<tmp2)   ?tmp1:tmp2;
          lowestT   = (tmp3<lowestT)?tmp3:lowestT;

    tValuesNeigh[invoc] = 0.0001 * lowestT;

    return false;

}

//----------------------------------------------------------------------------

void main()
{
    // use invocation ID to work out which agent is currently being computed
    int agentID = int(gl_GlobalInvocationID.x);

    // current agents position
    vec3 agentPos = texelFetch(neighbours,(agentID*3)).xyz;

    // current agents velocity
    vec3 agentVel = texelFetch(neighbours,(agentID*3) + 1).xyz;

    // current agents radius
    float agentRad = texelFetch(neighbours,(agentID*3) + 2).x;

    // current agents neighbour size
    int num_neigh = int(texelFetch(neighbours,(agentID*3) + 2).y);
    num_neigh =10;

    // start index into neighbour_ids for agents neighbours
    int start_neigh_index = int(texelFetch(neighbours,(agentID*3) + 2).z);

    // find current agents desired velocity
    vec3 agentDesVel = texelFetch(des_vel,agentID).xyz;

    //neighbour position array
    vec3 neighPos[10];

    //neighbour position array
    vec3 neighVel[10];

    //neighbour position array
    float neighRad[10];

    for(int i=0;i<num_neigh;i++)
    {
        int neigh_id = int(texelFetch(neighbour_ids,start_neigh_index+1).x);
        neighPos[i] = texelFetch(neighbours,neigh_id*3).xyz;
        neighVel[i] = texelFetch(neighbours,neigh_id*3 + 1).xyz;
        neighRad[i] = texelFetch(neighbours,neigh_id*3 + 2).x;
    }

    // create sample velocities
    vec3 test_velocity[72];
    createSampleVelocities(agentDesVel, test_velocity);

    //------------do FUNKY RVO stuff!----------------


    float tValuesVel[72];
    bool velInside = true;
    vec3 newVel;
    int velIndex = -1;

    int i=0;
    for(int i=0;i<72 /*&& velInside*/;i++)
    {
        newVel = test_velocity[i];
        float tValuesNeigh[10];
        bool velAccept[10];

        for(int j=0;j<num_neigh;j++)
        {
            //testVO against neighbour agents
            velAccept[j] = testVO(newVel,neighPos[j],neighVel[j],neighRad[j],
                                  agentPos,agentVel,agentRad,tValuesNeigh,j);


        }// end test neighbour loop

        bool accept = true;
        for(int j=0;j<10;j++)
        {
            if(!velAccept[j])
            {
                accept = false;
            }
        }

        // Lowest t value
        float lowestT = 100;
        if(accept)
        {
            // velocityis acceptable
            velInside = false;
            velIndex = i;
            i = 73; //1000 to be safe;
        }
        else
        {
            for(int j=0;j<num_neigh;j++)
            {
                if(tValuesNeigh[j]<lowestT && lowestT>0)
                {
                    lowestT = tValuesNeigh[j];
                }
            }
            tValuesVel[i] = lowestT;
            velIndex = i;
        }

        i++;
    }//end test velocity loop


    //find penalty values and chose best vel
    int index=-1;
    float penalty = 1000000000;
    float t = 1;
    if(!velInside)
    {
        //velocity acceptable
        new_vel[agentID] = vec4(newVel,0);
        //return;
    }
    else
    {
        for(int i=0;i<72;i++)
        {
            float tmpPenalty = (10/tValuesVel[i]);// + (m_desVel-testVelocities[i]).length();
            if(tmpPenalty < penalty && tmpPenalty>0)
            {
                penalty = tmpPenalty;
                t = tValuesVel[i];
                index = i;
            }
        }
        // set new velocity
        //newVel = 0.5 * (t*testVelocities[index] + m_agent->getOrigState().m_vel);
        newVel = t*test_velocity[index];

        // write results to newVel
        new_vel[agentID] = vec4(newVel,0);
    }
    return;
}
