#include "System.h"

#include <ngl/NGLStream.h>
#include <ngl/ShaderLib.h>
#include <ngl/Random.h>
#include <ngl/ShaderLib.h>
#include <ngl/Mat4.h>
#include <ngl/VAOPrimitives.h>
#include <boost/foreach.hpp>

#include "HashTable.h"

System::System(float _width)
{
    m_numAgents = 0;
    m_numBoundaries = 0;
    m_systemWidth = _width;

    //m_bounds = ngl::BBox(ngl::Vec3(0,0,0),20,20,20);
    BoundingBox bb;
    bb.m_minx = bb.m_miny = bb.m_minz = -10.0;
    bb.m_maxx = bb.m_maxy = bb.m_maxz = 10.0;

    m_octree = new AgentOctree (5, bb);

    m_hashTable = new HashTable(m_systemWidth,m_systemWidth,1.0,ngl::Vec3(0,0,0));
    m_hashTable->initVAO();
    setSpatialDivision(HASH);
    setBounds(ngl::BBox(ngl::Vec3(0,5,0),m_systemWidth,10,m_systemWidth));

    // setting up basic mesh from ngl::vaoprimitive
    ngl::VAOPrimitives *prim = ngl::VAOPrimitives::instance();
    prim->createCylinder("cylinder",0.1f,0.5f,4,1);

    m_globalGoal = ngl::Vec3(0.0f,0.0f,0.0f);

    initRVOCS();
}

System::~System()
{
    delete [] m_octree;
    delete [] m_hashTable;

    glDeleteTextures(1,&m_neighboursTEX);
    glDeleteBuffers(1,&m_neighboursBO);

    glDeleteTextures(1,&m_neighIdsTEX);
    glDeleteBuffers(1,&m_neighIdsBO);

    glDeleteBuffers(1,&m_desVelBO);

    glDeleteBuffers(1,&m_newVelSSBO);

    glDeleteVertexArrays(1,&m_rvoVAO);

}


typedef struct AGENTDATA{
    ngl::Vec3 pos;
    ngl::Vec3 vel;
    float rad;
    unsigned int numNeighbours;
    unsigned int startNeighIdIndex;
}AgentData;

void System::initRVOCS()
{
    ngl::ShaderLib *shader = ngl::ShaderLib::instance();

    shader->createShaderProgram("rvo");
    shader->attachShader("rvoCS",ngl::COMPUTE);
    shader->loadShaderSource("rvoCS","shaders/rvoCS.glsl");
    shader->compileShader("rvoCS");
    shader->attachShaderToProgram("rvo","rvoCS");
    shader->linkProgramObject("rvo");
    shader->use("rvo");

    glGenVertexArrays(1,&m_rvoVAO);
    glBindVertexArray(m_rvoVAO);



    // -------setting up neighbour sampler buffer-------
    glGenBuffers(1,&m_neighboursBO);
    glBindBuffer(GL_TEXTURE_BUFFER,m_neighboursBO);
    glBufferData(GL_TEXTURE_BUFFER,m_numAgents*sizeof(AgentData),NULL,GL_STATIC_DRAW);
    glGenTextures(1,&m_neighboursTEX);
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_BUFFER,m_neighboursTEX);
    glTexBuffer(GL_TEXTURE_BUFFER,GL_RGB32F, m_neighboursBO);
    glUniform1i(glGetUniformLocation(shader->getProgramID("rvo"), "neighbours"), 0);

    // -------setting up neighbour ID sampler buffer---------
    // This is an array of ints holding the ID of agents,
    // Stored in an order such that an agent can locate it's neighbours ID,
    // by accessing the current_agent_id*10 if each agent has 10 neighbours
    // alternative is to access at current_agent.startNeighIdIndex and end at +=numNeigh
    glGenBuffers(1,&m_neighIdsBO);
    glBindBuffer(GL_TEXTURE_BUFFER,m_neighIdsBO);
    glBufferData(GL_TEXTURE_BUFFER,NULL,NULL,GL_DYNAMIC_DRAW);
    glGenTextures(1,&m_neighIdsTEX);
    glActiveTexture(GL_TEXTURE1);
    glBindTexture(GL_TEXTURE_BUFFER,m_neighIdsTEX);
    glTexBuffer(GL_TEXTURE_BUFFER,GL_R32F, m_neighboursBO);
    glUniform1i(glGetUniformLocation(shader->getProgramID("rvo"), "neighbour_ids"), 1);


    // -------setting up agent buffer attribute--------
    glGenBuffers(1,&m_desVelBO);
    glBindBuffer(GL_TEXTURE_BUFFER,m_desVelBO);
    glBufferData(GL_TEXTURE_BUFFER,m_numAgents*sizeof(ngl::Vec3),NULL,GL_STATIC_DRAW);
    glGenTextures(1,&m_desVelTEX);
    glActiveTexture(GL_TEXTURE2);
    glBindTexture(GL_TEXTURE_BUFFER,m_desVelTEX);
    glTexBuffer(GL_TEXTURE_BUFFER,GL_RGB32F, m_desVelBO);
    glUniform1i(glGetUniformLocation(shader->getProgramID("rvo"), "des_vel"), 2);




    // ---------setting up shader storage buffer for new computed velocity--------
    glGenBuffers(1,&m_newVelSSBO);


    glBindVertexArray(0);


}

void System::updateRVOCS()
{
  ngl::ShaderLib *shader = ngl::ShaderLib::instance();
  shader->use("rvo");

    std::vector<AgentData> neighbours;
    std::vector<int> neighbourIDs;
    std::vector<ngl::Vec3> desVels;

    unsigned int _startNeighIdIndex;
    unsigned int _numNeighbours;
    BOOST_FOREACH(Agent *a,m_agents)
    {
        _startNeighIdIndex = neighbours.size();
        _numNeighbours = a->getBrain()->getNeighbours().size();

        // add agent to neighbour vector
        AgentData ad;
        ad.pos = a->getOrigState().m_pos;
        ad.vel = a->getOrigState().m_vel;
        ad.rad = a->getOrigState().m_rad;
        ad.numNeighbours = _numNeighbours;
        ad.startNeighIdIndex = _startNeighIdIndex;
        neighbours.push_back(ad);

        // ad agents desired velocity to attribute buffer
        desVels.push_back(a->getBrain()->getDesVel());


        for(unsigned int i=0;i<_numNeighbours;i++)
        {
            // add neighbour ids to neighbourIDs vector
            neighbourIDs.push_back(a->getBrain()->getNeighbours()[i]->getID());
        }

    }

    // bind vertex array
    glBindVertexArray(m_rvoVAO);


    // update neighbour buffer
    glBindBuffer(GL_TEXTURE_BUFFER,m_neighboursBO);
    glBufferData(GL_TEXTURE_BUFFER,m_numAgents*sizeof(AgentData),&neighbours[0].pos.m_x,GL_STATIC_DRAW);

    // update neighbour ID buffer
    glBindBuffer(GL_TEXTURE_BUFFER,m_neighIdsBO);
    glBufferData(GL_TEXTURE_BUFFER,neighbourIDs.size()*sizeof(int),&neighbourIDs[0],GL_STATIC_DRAW);

    // update desired velocity buffer
    glBindBuffer(GL_TEXTURE_BUFFER,m_desVelBO);
    glBufferData(GL_TEXTURE_BUFFER,m_numAgents*sizeof(ngl::Vec3),&desVels[0].m_x,GL_STATIC_DRAW);
    glUniform1i(glGetUniformLocation(shader->getProgramID("rvo"), "des_vel"), 2);


    // bind shader storage buffer
    glBindBufferBase(GL_SHADER_STORAGE_BUFFER,0,m_newVelSSBO);
    glBufferData(GL_SHADER_STORAGE_BUFFER,m_numAgents*sizeof(ngl::Vec4),NULL,GL_STATIC_DRAW);

    // call compute shader
    glDispatchCompute((m_numAgents/16)+m_numAgents%16,1,1);
    glBindBufferBase(GL_SHADER_STORAGE_BUFFER,0,0);
    glBindBuffer(GL_TEXTURE_BUFFER,0);
    // This makes sure all GPU stuff is done
    glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT);


    glBindBuffer(GL_ARRAY_BUFFER, m_newVelSSBO);
    // want to make buffer readable
    GLint bufAccess = GL_MAP_READ_BIT;
    ngl::Vec4 *newVels = (ngl::Vec4 *)glMapBufferRange(GL_ARRAY_BUFFER,
                                                       0,
                                                       m_numAgents*sizeof(ngl::Vec4),
                                                       bufAccess);

    if(newVels == (ngl::Vec4 *)NULL){std::cout<<"mapping buffer failed\n";}

    ngl::Vec4 result[m_numAgents];
    memcpy(result,newVels,m_numAgents*sizeof(ngl::Vec4));

    glUnmapBuffer(GL_ARRAY_BUFFER);
    glBindBuffer(GL_ARRAY_BUFFER,0);

//    for(int i=0;i<m_numAgents;i++)
//    {
//        std::cout<<m_agents[i]->getOrigState().m_vel<<"\n";
//        std::cout<<result[i]<<"\n";
//        //m_agents[i]->setVel(result[i]);
//    }


    glBindVertexArray(0);

}

void System::update()
{
    addNeighbours();
    addBoundaries();


    // call compute shader here
    updateRVOCS();

    BOOST_FOREACH( Agent* a, m_agents)
    {
        a->update();
    }



    clearNeighbours();
    clearBoundaries();

    BOOST_FOREACH( Agent* a, m_agents)
    {
        a->updateState();
    }
}

void System::addAgent(const Avoidance &_avoidType)
{
    m_agents.push_back(new Agent(this,_avoidType,m_numAgents));
    m_numAgents++;
}

void System::addNeighbours()
{
  switch (m_spatialDivision)
    {
    case BRUTE :
      BOOST_FOREACH( Agent* a, m_agents)
      {
          BOOST_FOREACH( Agent* b, m_agents)
          {
              if(a == b){continue;}
              a->getBrain()->addNeighbour(b);
          }
      }
      break;

    case OCTREE :
      m_octree->clearTree();
      BOOST_FOREACH( Agent* a, m_agents)
      {
          m_octree->addObject(a);
      }
      m_octree->addNeighbours();
      break;

    case HASH :
      updateHashTable();
      m_hashTable->addNeighbours();


      break;
    }
}

void System::addBoundaries()
{
    switch (m_spatialDivision)
      {
      case BRUTE :
        break;

      case OCTREE :
        break;

    case HASH :
        m_hashTable->addBoundaryToAgent();
        break;
      }

}

void System::clearNeighbours()
{
    BOOST_FOREACH( Agent* a, m_agents)
    {
        a->getBrain()->clearNeighbours();
    }
}

void System::clearBoundaries()
{
    BOOST_FOREACH( Agent* a, m_agents)
    {
        a->getBrain()->clearBoundary();
    }
}

void System::setSpatialDivision(const SpatialDivision &_type)
{
  m_spatialDivision = _type;
  switch (m_spatialDivision)
    {
    case BRUTE :
      break;

    case OCTREE :
      break;

  case HASH :
      updateHashTable();
      break;
    }
}

void System::setBounds(ngl::BBox _bounds)
{
    m_bounds = _bounds;

    float left,right,bottom,top;
    left   = m_bounds.center().m_x - (0.5*m_bounds.width());
    right  = m_bounds.center().m_x + (0.5*m_bounds.width());
    bottom = m_bounds.center().m_z - (0.5*m_bounds.depth());
    top    = m_bounds.center().m_z + (0.5*m_bounds.depth());

    ngl::Vec3 BL,TL,BR,TR;
    BL = ngl::Vec3(left, 0,bottom);
    TL = ngl::Vec3(left, 0,top);
    BR = ngl::Vec3(right,0,bottom);
    TR = ngl::Vec3(right,0,top);

    //m_Boundaries.push_back(new Boundary(TL,TR,BR,BL,false));

    /*
    // left side
    m_Boundaries.push_back(new Boundary(TL,BL));
    m_hashTable->addBoundaryToHash(m_Boundaries[0]);
    // right side
    m_Boundaries.push_back(new Boundary(BR,TR));
    m_hashTable->addBoundaryToHash(m_Boundaries[1]);
    // top side
    m_Boundaries.push_back(new Boundary(TR,TL));
    m_hashTable->addBoundaryToHash(m_Boundaries[2]);
    // bottom side
    m_Boundaries.push_back(new Boundary(BL,BR));
    m_hashTable->addBoundaryToHash(m_Boundaries[3]);
    */

    //m_numBoundaries += 4;
}

void System::addBounds(Boundary *_boundary)
{
    m_Boundaries.push_back(_boundary);
    m_hashTable->addBoundaryToHash(m_Boundaries.back());
}

void System::addBounds4(Boundary4 * _boundary)
{
    m_Bounds4.push_back(_boundary);
}

void System::setGloablGoal(const ngl::Vec3 &_goal)
{
    m_globalGoal = _goal;
    BOOST_FOREACH(Agent* a,m_agents)
    {
        a->getBrain()->setGoal(_goal);
    }
}

void System::setRandomGoal()
{
    BOOST_FOREACH(Agent* a,m_agents)
    {
        ngl::Random *r = ngl::Random::instance();
        ngl::Vec3 goal = r->getRandomVec3()*0.5*(m_systemWidth-1);
        a->getBrain()->setGoal(goal);
    }
}

void System::setGlobalSocialAttract(const float &_socialAttract)
{
  BOOST_FOREACH(Agent* a,m_agents)
  {
      a->getBrain()->setSocialAttraction(_socialAttract);
  }
}

void System::toggleSocialAttract()
{
  std::cout<<"toggling social attract coeef\n";
  static int i;
  i++;
  int event = i%3;
  if(event == 0)
    {
      BOOST_FOREACH(Agent* a,m_agents)
      {
          a->getBrain()->setSocialAttraction(0.0f);
      }
    }
  else if(event == 1)
    {
      BOOST_FOREACH(Agent* a,m_agents)
      {
          a->getBrain()->setSocialAttraction(0.0001f);
      }
    }
  else if(event == 2)
    {
      BOOST_FOREACH(Agent* a,m_agents)
      {
          a->getBrain()->setSocialAttraction(-0.0001f);
      }
    }

}

void System::loadMatricesToShader()
{
  ngl::ShaderLib *shader=ngl::ShaderLib::instance();

  ngl::Mat4 MV;
  ngl::Mat4 MVP;
  ngl::Mat3 normalMatrix;
  ngl::Mat4 M;

  M   = m_globalTX;
  MV  = M*m_cam.getViewMatrix();
  MVP = M*m_cam.getVPMatrix();
  normalMatrix=MV;
  normalMatrix.inverse();

  shader->setShaderParamFromMat4("MV",MV);
  shader->setShaderParamFromMat4("MVP",MVP);
  shader->setShaderParamFromMat3("normalMatrix",normalMatrix);
  shader->setShaderParamFromMat4("M",M);
}

void System::draw()
{
    loadMatricesToShader();
    m_bounds.recalculate();
    m_bounds.draw();
    m_hashTable->draw();
    BOOST_FOREACH(Boundary* b, m_Boundaries)
    {
      b->loadMatricesToShader(m_globalTX,m_cam);
      b->draw();
    }
    BOOST_FOREACH(Agent* a, m_agents)
    {
        a->loadMatricesToShader();
        a->draw();
    }
}

void System::setUpDraw(const ngl::Camera &_cam, const ngl::Mat4 &_tx)
{
    m_cam = _cam;
    m_globalTX = _tx;
}

ngl::Camera System::getCam()const
{
    return m_cam;
}
ngl::Mat4 System::getGlobalTX()const
{
    return m_globalTX;
}

float System::getSystemWidth() const
{
    return m_systemWidth;
}



void System::updateHashTable()
{
    BOOST_FOREACH( Agent* a, m_agents)
    {
        Cell *newCell = m_hashTable->getCell(*a);
        if(newCell != a->getCell())
        {
            m_hashTable->removeAgent(a);
            m_hashTable->addAgent(a,newCell);
        }
    }
}


void System::setScene(const int &_scene)
{
  m_scene = _scene;
  if(m_scene == 0)
  {     //circle scene
      int numAgentsPerCircle = 100;
      int numCircles = (m_numAgents / numAgentsPerCircle);
      int remainderAgents = m_numAgents % numAgentsPerCircle;
      int index = 0;
      for(int i=0;i<numCircles;i++)
      {
          for(int j=0;j<100;j++)
          {
              float rad = m_systemWidth * 0.5 - i;
              ngl::Vec3 pos = ngl::Vec3(rad*sin((j*6.28)/numAgentsPerCircle),0,rad*cos((j*6.28)/numAgentsPerCircle));
              m_agents[index]->setPos(pos);
              m_agents[index]->setGoal(-pos);

              index++;
          }
      }
      int lastCircle = remainderAgents;
      for(int j=0;j<lastCircle;j++)
      {
          float rad = m_systemWidth * 0.5 - numCircles;
          ngl::Vec3 pos = ngl::Vec3(rad*sin((j*6.28)/lastCircle),0,rad*cos((j*6.28)/lastCircle));
          m_agents[index]->setPos(pos);
          m_agents[index]->setGoal(-pos);

          index++;
      }

      return;
  }
  else if(m_scene == 1)
  {
      for(int i=0;i<m_numAgents;i++)
      {
          ngl::Random *r = ngl::Random::instance();
          ngl::Vec3 pos = (m_systemWidth-1) * 0.5 * r->getRandomVec3();
          m_agents[i]->setPos(pos);
      }
      setRandomGoal();
      return;
  }
  else if(m_scene == 2)
  {
    if(m_numAgents > 3)
    {
        int numGroup = (int)(m_numAgents / 4);
        for(int i=0;i<m_numAgents;i++)
        {
            ngl::Random *r = ngl::Random::instance();
            if(i<numGroup)
            {
                ngl::Vec3 pos = (m_systemWidth-1) * 0.1 * r->getRandomVec3() - ngl::Vec3(-m_systemWidth*0.4,0,0);
                m_agents[i]->setPos(pos);
                m_agents[i]->setGoal(-pos);
            }
            else if(i<2*numGroup)
            {
                ngl::Vec3 pos = (m_systemWidth-1) * 0.1 * r->getRandomVec3() - ngl::Vec3(m_systemWidth*0.4,0,0);
                m_agents[i]->setPos(pos);
                m_agents[i]->setGoal(-pos);
            }
            else if(i<3*numGroup)
            {
                ngl::Vec3 pos = (m_systemWidth-1) * 0.1 * r->getRandomVec3() - ngl::Vec3(0,0,-m_systemWidth*0.4);
                m_agents[i]->setPos(pos);
                m_agents[i]->setGoal(-pos);
            }
            else
            {
                ngl::Vec3 pos = (m_systemWidth-1) * 0.1 * r->getRandomVec3() - ngl::Vec3(0,0,m_systemWidth*0.4);
                m_agents[i]->setPos(pos);
                m_agents[i]->setGoal(-pos);
            }
        }
    }
    return;
  }


  updateHashTable();
}

void System::printInfo()const
{
    m_hashTable->printInfo();
}
