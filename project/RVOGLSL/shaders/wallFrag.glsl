#version 400 core
/// @brief our output fragment colour
layout (location =0) out vec4 fragColour;
/// @brief[in] the vertex normal

void main ()
{
    fragColour = vec4(1.0f,0.0f,0.0f,1.0f);
}

