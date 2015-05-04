#version 440 core
// num threads in workgroup
layout (local_size_x = 3,local_size_y = 3,local_size_z = 1)

// hash table cell buckets ids,
// each cell has an id that is the id for the associated bucket of agents
uniform sampler2D hash_cell;

// array of buckets with
uniform sampler2D hash_bucket;

void main()
{

}
