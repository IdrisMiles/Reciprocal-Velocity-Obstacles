#ifndef _BRAIN__H_
#define _BRAIN__H_

#include <vector>
#include<boost/shared_ptr.hpp>
#include <ngl/Vec3.h>
#include <ngl/Mat4.h>
#include <ngl/VertexArrayObject.h>

class System;
class Agent;
class Boundary;

enum Avoidance {FLOCKING,RVO,SOCIAL};

class Brain
{
public:

    Brain(Agent *_agent, System *_system);
    ~Brain();

    void update();
    void setSystem(System *_system);
    void setAvoidanceType(const Avoidance &_avoidance);
    void setGoal(const ngl::Vec3 &_goal);
    void mapRoute();
    void findNextGoal();
    void findNeighbours();
    void findBoundaries();

    void rvo();
    void initVoVAO();
    void loadMatricesToShader();

    std::vector<ngl::Vec3> createSampleVel()const;
    /// @brief method to check if a point lies on the left hand side of an edge
    /// @param [in] const ngl::Vec3 &_point this is the point being tested
    /// @param [in] const ngl::Vec3 &_edge direction vector of line segment
    /// @param [in] const ngl::Vec3 &_edgePoint starting point of edge(line segment)
    /// @return bool, true if point left of edge, false if point right of edge
    bool pointLeftOfEdge(const ngl::Vec3 &_point,
                         const ngl::Vec3 &_edge, const ngl::Vec3 &_edgePoint)const;

    /// @brief method to check if two lines segments intersect
    /// @param [in] const ngl::Vec3 &_vel this is a direction vector from _p1
    /// @param [in] const ngl::Vec3 &_p1 this is the start point of the first line segment
    /// @param [in] const ngl::Vec3 &_edge this is a direction vector from _p2
    /// @param [in] const ngl::Vec3 &_p2 this is the start point of the second line segment
    /// @return float , this is the t value for the first line segment,
    /// how long along the segment intersection occurs. \n
    /// If no intersection returns -1
    float checkIntersection(const ngl::Vec3 &_vel,const ngl::Vec3 &_p1,
                            const ngl::Vec3 &_edge, const ngl::Vec3 &_p2) const;
    /// @brief method to find the determinant of 2 2D vectors - cross product of 2D vectors
    /// @param [in] const ngl::Vec3 &_v1
    /// @param [in] const ngl::Vec3 &_v2
    /// @return float , this is the value of the determinant.
    float det2D(const ngl::Vec3 &_v1,const ngl::Vec3 &_v2)const;
    ngl::Vec3 normal2D(const ngl::Vec3 &_vec)const;
    ngl::Vec3 matXvec(const ngl::Mat4 &_mat,const ngl::Vec3 &_pos)const;

    void flocking();
    ngl::Vec3 flockSeparation();
    ngl::Vec3 flockCohesion();
    ngl::Vec3 flockAlignment();
    ngl::Vec3 flockGoal();

    void socialForces();


    float getPerceiveRad()const;
    float getPerceiveAng()const;
    float getDesSpeed()const;

    void clearNeighbours();
    void clearBoundary();
    void addNeighbour(Agent* _neighbour);
    void addBoundary(Boundary *_boundary);

    void printInfo();

private:

    System *m_system;
    Agent *m_agent;
    Avoidance m_avoidanceType;

    std::vector< Agent* > m_neighbours;
    std::vector<Boundary*> m_Boundaries;


    ngl::Vec3 m_desVel;
    float m_desSpeed;
    float m_maxVel;
    float m_maxAcc;
    //ngl::Vec3 m_newVel;
    ngl::Vec3 m_goal;
    ngl::Vec3 m_nextGoal;
    std::vector<ngl::Vec3> m_wayPoints;

    float m_perceiveRad;
    float m_perceiveAng;

    //flocking stuff
    float m_goalWeight;
    float m_alignmentWeight;
    float m_cohesionWeight;
    float m_separationWeight;

    ngl::VertexArrayObject *m_voVAO;

};



#endif //_BRAIN__H_
