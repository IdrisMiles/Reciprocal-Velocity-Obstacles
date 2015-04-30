#include <QMouseEvent>
#include <QGuiApplication>

#include "NGLScene.h"
#include <ngl/NGLStream.h>
#include <ngl/Camera.h>
#include <ngl/Light.h>
#include <ngl/Material.h>
#include <ngl/NGLInit.h>
#include <ngl/VAOPrimitives.h>
#include <ngl/ShaderLib.h>


//----------------------------------------------------------------------------------------------------------------------
/// @brief the increment for x/y translation with mouse movement
//----------------------------------------------------------------------------------------------------------------------
const static float INCREMENT=0.01;
//----------------------------------------------------------------------------------------------------------------------
/// @brief the increment for the wheel zoom
//----------------------------------------------------------------------------------------------------------------------
const static float ZOOM=0.1;

NGLScene::NGLScene(QWindow *_parent) : OpenGLWindow(_parent)
{
  // re-size the widget to that of the parent (in this case the GLFrame passed in on construction)
  m_rotate=false;
  // mouse rotation values set to 0
  m_spinXFace=0;
  m_spinYFace=0;
  setTitle("Qt5 Simple NGL Demo");
  m_scene = 0;
  m_pause = true;
  m_editMode = false;

  //fps stuff
  m_startClock = std::clock();
  m_fps=0;
  m_frames=0;
 
}


NGLScene::~NGLScene()
{
  ngl::NGLInit *Init = ngl::NGLInit::instance();
  std::cout<<"Shutting down NGL, removing VAO's and Shaders\n";
  delete m_light;
  Init->NGLQuit();
}

void NGLScene::resizeEvent(QResizeEvent *_event )
{
  if(isExposed())
  {
  // set the viewport for openGL we need to take into account retina display
  // etc by using the pixel ratio as a multiplyer
  glViewport(0,0,width()*devicePixelRatio(),height()*devicePixelRatio());
  // now set the camera size values as the screen size has changed
  m_cam->setShape(45,(float)width()/height(),0.05,350);
  renderLater();
  }
}


void NGLScene::initialize()
{
  // we must call this first before any other GL commands to load and link the
  // gl commands from the lib, if this is not done program will crash
  ngl::NGLInit::instance();

  glClearColor(0.4f, 0.4f, 0.4f, 1.0f);			   // Grey Background
  // enable depth testing for drawing
  glEnable(GL_DEPTH_TEST);
  // enable multisampling for smoother drawing
  glEnable(GL_MULTISAMPLE);

  //---------------Shader setup-----------------------
  // now to load the shader and set the values
  // grab an instance of shader manager
  ngl::ShaderLib *shader=ngl::ShaderLib::instance();

  //=============PHONG====================
  // we are creating a shader called Phong
  shader->createShaderProgram("Phong");

  // now we are going to create empty shaders for Frag and Vert
  shader->attachShader("PhongVertex",ngl::VERTEX);
  shader->attachShader("PhongFragment",ngl::FRAGMENT);

  // attach the source
  shader->loadShaderSource("PhongVertex","shaders/PhongVertex.glsl");
  shader->loadShaderSource("PhongFragment","shaders/PhongFragment.glsl");

  // compile the shaders
  shader->compileShader("PhongVertex");
  shader->compileShader("PhongFragment");

  // add them to the program
  shader->attachShaderToProgram("Phong","PhongVertex");
  shader->attachShaderToProgram("Phong","PhongFragment");

  // now bind the shader attributes for most NGL primitives we use the following
  // layout attribute 0 is the vertex data (x,y,z)
  shader->bindAttribute("Phong",0,"inVert");
  // attribute 1 is the UV data u,v (if present)
  shader->bindAttribute("Phong",1,"inUV");
  // attribute 2 are the normals x,y,z
  shader->bindAttribute("Phong",2,"inNormal");

  // now we have associated this data we can link the shader
  shader->linkProgramObject("Phong");
  // and make it active ready to load values
  (*shader)["Phong"]->use();

  //==============WALL SHADER=================
  shader->createShaderProgram("Wall");

  shader->attachShader("wallVert",ngl::VERTEX);
  shader->attachShader("wallFrag",ngl::FRAGMENT);

  shader->loadShaderSource("wallVert","shaders/wallVert.glsl");
  shader->loadShaderSource("wallFrag","shaders/wallFrag.glsl");

  shader->compileShader("wallVert");
  shader->compileShader("wallFrag");

  shader->attachShaderToProgram("Wall","wallVert");
  shader->attachShaderToProgram("Wall","wallFrag");
  shader->bindAttribute("Wall",0,"inVert");
  shader->bindAttribute("Wall",1,"inUV");
  shader->bindAttribute("Wall",2,"inNormal");
  shader->linkProgramObject("Wall");
  //shader->use("Wall");


  // Now we will create a basic Camera from the graphics library
  // This is a static camera so it only needs to be set once
  // First create Values for the camera position
  ngl::Vec3 from(0,0,10);
  ngl::Vec3 to(0,0,0);
  ngl::Vec3 up(0,1,0);
  // now load to our new camera
  m_cam= new ngl::Camera(from,to,up);
  // set the shape using FOV 45 Aspect Ratio based on Width and Height
  // The final two are near and far clipping planes of 0.5 and 10
  m_cam->setShape(45,(float)720.0/576.0,0.05,350);
  shader->setShaderParam3f("viewerPos",m_cam->getEye().m_x,m_cam->getEye().m_y,m_cam->getEye().m_z);
  // now create our light this is done after the camera so we can pass the
  // transpose of the projection matrix to the light to do correct eye space
  // transformations
  ngl::Mat4 iv=m_cam->getViewMatrix();
  iv.transpose();
  m_light = new ngl::Light(ngl::Vec3(0,-5,10),ngl::Colour(1,1,1,1),ngl::Colour(1,1,1,1),ngl::POINTLIGHT );
  m_light->setTransform(iv);
  // load these values to the shader as well
  m_light->loadToShader("light");
  // as re-size is not explicitly called we need to do this.
  // set the viewport for openGL we need to take into account retina display
  // etc by using the pixel ratio as a multiplyer
  glViewport(0,0,width()*devicePixelRatio(),height()*devicePixelRatio());

  m_text = new ngl::Text(QFont("Arial",14));
  m_text->setScreenSize(width(),height());

  m_system = new System(40.0f);

  for(int i=0;i<4;i++)
  {
      //m_system->addAgent(FLOCKING);
      m_system->addAgent(RVO);
      //m_system->addAgent(SOCIAL);
  }

  startTimer(10);
}

void NGLScene::update()
{
    if(!m_pause){return;}
    m_system->update();
}

void NGLScene::editMode()
{
    m_pause = false;
    m_editMode = true;
}

void NGLScene::pausePlaySim()
{
    m_pause = !m_pause;
    m_editMode = false;
}

void NGLScene::toggleScene()
{
    m_editMode = false;
    m_system->setScene((m_scene++)%3);
}

//void NGLScene::loadMatricesToShader()
//{
//  ngl::ShaderLib *shader=ngl::ShaderLib::instance();

//  ngl::Mat4 MV;
//  ngl::Mat4 MVP;
//  ngl::Mat3 normalMatrix;
//  ngl::Mat4 M;

//  M   = m_mouseGlobalTX;
//  MV  = M*m_cam->getViewMatrix();
//  MVP = M*m_cam->getVPMatrix();
//  normalMatrix=MV;
//  normalMatrix.inverse();

//  shader->setShaderParamFromMat4("MV",MV);
//  shader->setShaderParamFromMat4("MVP",MVP);
//  shader->setShaderParamFromMat3("normalMatrix",normalMatrix);
//  shader->setShaderParamFromMat4("M",M);
//}

void NGLScene::render()
{
  // clear the screen and depth buffer
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  // grab an instance of the shader manager
  ngl::ShaderLib *shader=ngl::ShaderLib::instance();
  (*shader)["Phong"]->use();

  //-----------fps counter---------
  ++m_frames;
  if(m_pause){std::cout<<"FPS: "<<m_fps<<"\n";}
  //m_text->setColour(1,1,0);
//  QString text=QString("FPS: %2 ").arg(m_fps);
//  m_text->renderText(10,20,text);

  //-------------Global TX matrix sent to Agents---------------
  // Rotation based on the mouse position for our global transform
  ngl::Mat4 rotX;
  ngl::Mat4 rotY;
  // create the rotation matrices
  rotX.rotateX(m_spinXFace);
  rotY.rotateY(m_spinYFace);
  // multiply the rotations
  m_mouseGlobalTX=rotY*rotX;
  // add the translations
  m_mouseGlobalTX.m_m[3][0] = m_modelPos.m_x;
  m_mouseGlobalTX.m_m[3][1] = m_modelPos.m_y;
  m_mouseGlobalTX.m_m[3][2] = m_modelPos.m_z;

  if(m_editMode)
  {
      ngl::Mat4 rot;
      rot.rotateX(90);
      // multiply the rotations
      m_mouseGlobalTX=rot;
      // add the translations
      m_mouseGlobalTX.m_m[3][0] = 0.0f;
      m_mouseGlobalTX.m_m[3][1] = 0.0f;
      m_mouseGlobalTX.m_m[3][2] = -15.0f;
  }

  //---------System drawing methods------------
  m_system->setUpDraw(*m_cam,m_mouseGlobalTX);
  m_system->draw();

}

//----------------------------------------------------------------------------------------------------------------------
void NGLScene::mouseMoveEvent (QMouseEvent * _event)
{
  // note the method buttons() is the button state when event was called
  // this is different from button() which is used to check which button was
  // pressed when the mousePress/Release event is generated
  if(m_rotate && !m_editMode && _event->buttons() == Qt::LeftButton)
  {
    int diffx=_event->x()-m_origX;
    int diffy=_event->y()-m_origY;
    m_spinXFace += (float) 0.5f * diffy;
    m_spinYFace += (float) 0.5f * diffx;
    m_origX = _event->x();
    m_origY = _event->y();
    renderLater();
  }
  else if(m_editMode && _event->buttons() == Qt::LeftButton)
  {
      //edit mode, draw boundaries.

      m_endXbound = (float)(_event->x() - 0.5*width())/width();
      m_endYbound = (float)(_event->y() - 0.5*height())/height();

  }
        // right mouse translate code
  else if(m_translate && !m_editMode && _event->buttons() == Qt::RightButton)
  {
    int diffX = (int)(_event->x() - m_origXPos);
    int diffY = (int)(_event->y() - m_origYPos);
    m_origXPos=_event->x();
    m_origYPos=_event->y();
    m_modelPos.m_x += INCREMENT * diffX;
    m_modelPos.m_y -= INCREMENT * diffY;
    renderLater();

   }
}


//----------------------------------------------------------------------------------------------------------------------
void NGLScene::mousePressEvent ( QMouseEvent * _event)
{
  // this method is called when the mouse button is pressed in this case we
  // store the value where the maouse was clicked (x,y) and set the Rotate flag to true
  if(_event->button() == Qt::LeftButton)
  {
    m_origX = _event->x();
    m_origY = _event->y();
    m_rotate =true;

    if(m_editMode)
    {
        m_origXbound = (float)(_event->x() - 0.5*width())/width();
        m_origYbound = (float)(_event->y() - 0.5*height())/height();
    }
  }
  // right mouse translate mode
  else if(_event->button() == Qt::RightButton)
  {
    m_origXPos = _event->x();
    m_origYPos = _event->y();
    m_translate=true;
  }

}

//----------------------------------------------------------------------------------------------------------------------
void NGLScene::mouseReleaseEvent ( QMouseEvent * _event )
{
  // this event is called when the mouse button is released
  // we then set Rotate to false
  if (_event->button() == Qt::LeftButton)
  {
    m_rotate=false;
    if(m_editMode)
    {
        std::cout<<"edit mode, about to add bound\n";

        // orig x = minX
        // end x = maxX
        // orig y = maxY
        // end y = minY
        /*  minX,minY---------------maxX,minY
         *  |                           |
         *  |                           |
         *  |                           |
         *  minX,maxY---------------MaxX,MaxY
         *
         *  minX,maxY---------------maxX,maxY
         *  |                           |
         *  |                           |
         *  |                           |
         *  minX,minY---------------MaxX,MinY
         *
         */
        ngl::Vec3 p0 = 20 * ngl::Vec3(m_origXbound,0,m_origYbound);
        ngl::Vec3 p1 = 20 * ngl::Vec3(m_endXbound,0,m_origYbound);
        ngl::Vec3 p2 = 20 * ngl::Vec3(m_endXbound,0,m_endYbound);
        ngl::Vec3 p3 = 20 * ngl::Vec3(m_origXbound,0,m_endYbound);
        m_system->addBounds(new Boundary(p0,p1,p2,p3,true));
    }
  }
        // right mouse translate mode
  if (_event->button() == Qt::RightButton)
  {
    m_translate=false;
  }
}

//----------------------------------------------------------------------------------------------------------------------
void NGLScene::wheelEvent(QWheelEvent *_event)
{

    if(m_editMode){return;}
	// check the diff of the wheel position (0 means no change)
	if(_event->delta() > 0)
	{
		m_modelPos.m_z+=ZOOM;
	}
	else if(_event->delta() <0 )
	{
		m_modelPos.m_z-=ZOOM;
	}
	renderLater();
}
//----------------------------------------------------------------------------------------------------------------------

void NGLScene::keyPressEvent(QKeyEvent *_event)
{
  // this method is called every time the main window recives a key event.
  // we then switch on the key value and set the camera in the GLWindow
  switch (_event->key())
  {
  // escape key to quite
  case Qt::Key_Escape : QGuiApplication::exit(EXIT_SUCCESS); break;
  // turn on wirframe rendering
  case Qt::Key_W : glPolygonMode(GL_FRONT_AND_BACK,GL_LINE); break;
  // turn off wire frame
  case Qt::Key_S : glPolygonMode(GL_FRONT_AND_BACK,GL_FILL); break;
  // show full screen
  case Qt::Key_F : showFullScreen(); break;
  // show windowed
  case Qt::Key_N : showNormal(); break;
  case Qt::Key_P : pausePlaySim(); break;
  case Qt::Key_E : editMode();break;
  case Qt::Key_O : m_system->setSpatialDivision(OCTREE);break;
  case Qt::Key_U : m_system->setSpatialDivision(BRUTE);break;
  case Qt::Key_I : m_system->setSpatialDivision(HASH);break;
  case Qt::Key_Q : m_system->toggleSocialAttract();
  case Qt::Key_Space : m_system->printInfo();break;

  case Qt::Key_Z : m_system->setGloablGoal(ngl::Vec3(1.0f,0.0f,-5.0f)); break;
  case Qt::Key_X : m_system->setRandomGoal(); break;

  case Qt::Key_Tab : toggleScene();break;

  case Qt::Key_1 : m_system->addAgent(FLOCKING);break;
  case Qt::Key_2 : m_system->addAgent(RVO);break;
  case Qt::Key_3 : m_system->addAgent(SOCIAL);break;
  default : break;
  }
  // finally update the GLWindow and re-draw
  //if (isExposed())
    renderLater();
}


void NGLScene::timerEvent(QTimerEvent *_event)
{
    update();

    m_timeElapsed = ( std::clock() - m_startClock ) / (double) CLOCKS_PER_SEC;
    if(m_timeElapsed >= 0.5)
    {
        //reset clock and frames
        m_startClock = std::clock();
        m_fps = m_frames / m_timeElapsed;
        m_frames = 0;
    }

    renderNow();
}
