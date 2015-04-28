#version 440 core

layout (local_size_x = 128) in;

uniform samplerBuffer desVel;

uniform samplerBuffer neighbours;       //texture location 0
uniform samplerBuffer neighbour_ids;    //texture location 1

// SSBO - destination for output
// uniform image1D or uniform



void main()
{
    //do RVO stuff!
    // use invocation ID to work out which agent is currently being computed




    // write results to newVel
}
