#version 440 core

layout (local_size_x = 16) in;

uniform samplerBuffer neighbours;       //texture location 0
uniform samplerBuffer neighbour_ids;    //texture location 1
uniform samplerBuffer des_vel;          //texture location 2


// SSBO - destination for output
layout (std430, binding = 0) buffer New_Vel_Buffer
{
    vec4 new_vel[];
};


void createSampleVelocities(in vec3 desVel, inout vec3 testVels[72])
{
    float speed = length(desVel);
    vec3 u_DesVel = desVel;
    normalize(u_DesVel);
    float theta = asin(u_DesVel.x);
    for(int i=0;i<36;i+=2)
    {
        testVels[i].xz    = speed * vec2(cos(theta+(i*5)),sin(theta+(i*5)));
        testVels[i+1].xz  = speed * vec2(cos(theta-(i*5)),sin(theta-(i*5)));
    }
}

bool pointLeftOfEdge(in vec3 point, in vec3 edge, in vec3 edgeOrig)
{
    vec2 tmp = vec2((point - edgeOrig).xz);
    mat2 det;
    det[0] = edge.xz;
    det[1] = tmp;
    float result = determinant(det);

    return (result >= 0);

}

float timeToIntersection(in vec3 vel, in vec3 p1, in vec3 edge, in vec3 p2, in float time)
{
    mat2 edges;
    edges[0] = vel.xz;
    edges[1] = edge.xz;
    float detVE = determinant(edges);

    if(detVE != 0)
    {
        //edges will collide

        vec3 dist = p2 - p1;
        mat2 edgeDist;
        edgeDist[0] = dist.xz;
        edgeDist[1] = edge.xz;
        float detED = determinant(edgeDist);
        float t = - detED / detVE;

        if(t>=0 && t<=time)
        {
            return t;
        }

    }

    return -1.0f;
}

bool testVO(in vec3 testVel, in vec3 neighPos, in vec3 neighVel, in float neighRad,
            in vec3 agentPos, in vec3 agentVel, in float agentRad, out float tValuesNeigh[10])
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
    float time = 100;
    float t1 = timeToCollision(testVel,agentPos,frontEdge,p4,time);
    float t2 = timeToCollision(testVel,agentPos,rightEdge,p5,time);
    float t3 = timeToCollision(testVel,agentPos,leftEdge,p4,time);


    return false;

}

void main()
{
    // use invocation ID to work out which agent is currently being computed
    int agentID = int(gl_GlobalInvocationID.x);

    // current agents position
    vec3 agentPos = texelFetch(neighbours,(agentID*3)).xyz;

    // current agents velocity
    vec3 agentVel = texelFetch(neighbours,(agentID*3) + 1).xyz;

    // current agents radius
    float rad = texelFetch(neighbours,(agentID*3) + 2).x;

    // current agents neighbour size
    int num_neigh = int(texelFetch(neighbours,(agentID*3) + 2).y);

    // start index into neighbour_ids for agents neighbours
    int start_neigh_index = int(texelFetch(neighbours,(agentID*3) + 2).z);

    // find current agents desired velocity
    vec3 agentDesVel = texelFetch(des_vel,agentID).xyz;


    // create sample velocities
    vec3 test_velocity[72];
    createSampleVelocities(agentDesVel, test_velocity);

    //------------do FUNKY RVO stuff!----------------


    float tValues[];
    bool velInside = true;

//    for(int i=0;i<72;i++)
//    {


//    }

    // write results to newVel
    new_vel[agentID] = vec4(agentVel,0);




}
