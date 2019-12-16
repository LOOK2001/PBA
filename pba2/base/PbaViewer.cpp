//-------------------------------------------------------
//
//  PbaViewer.C
//
//  This viewer is a wrapper of the Glut calls needed to
//  display opengl data.  Options for zooming, 
//  labeling the window frame, etc are available for 
//  derived classes to use.
//
//  There is a notion of PbaThings, which are abstract
//  entities that can do processing and handle glut-like
//  calls for Idle, Init, Display, and other API options.
 
//
//  Copyright (c) 2003,2017 Jerry Tessendorf
//
//--------------------------------------------------------

//#include <GL/glut.h>



#ifdef __APPLE__
  #include <OpenGL/gl.h>   // OpenGL itself.
  #include <OpenGL/glu.h>  // GLU support library.
  #include <GLUT/glut.h>
#else
  #include <Windows.h>
  #include <GL/gl.h>   // OpenGL itself.
  #include <GL/glu.h>  // GLU support library.
  #include <GL/glut.h> // GLUT support library.
#endif

#include <iostream>
#include <cmath>
#include "PbaViewer.h"

using namespace std;
namespace pba{


// These are the GLUT Callbacks that are implemented in PbaViewer and PbaThings.
void cbDisplayFunc()
{
   glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );	
   PbaViewer::Instance() -> Display();
   glutSwapBuffers();
   glutPostRedisplay();
}

void cbIdleFunc()
{
   PbaViewer::Instance() -> Idle();
}


void cbKeyboardFunc( unsigned char key, int x, int y )
{
   PbaViewer::Instance() -> Keyboard( key, x, y );
}

void cbMotionFunc( int x, int y )
{
   
   PbaViewer::Instance() -> Motion( x, y );
   glutPostRedisplay();
}

void cbPassiveMotionFunc(int x, int y)
{
	PbaViewer::Instance()->PassiveMotion(x, y);
	glutPostRedisplay();
}

void cbMouseFunc( int button, int state, int x, int y )
{
   PbaViewer::Instance() -> Mouse( button, state, x, y );
}

void cbReshapeFunc( int w, int h )
{
   PbaViewer::Instance() -> Reshape( w, h );
   glutPostRedisplay();
}


PbaViewer* PbaViewer::pPbaViewer = nullptr;
	
PbaViewer::PbaViewer() : 
   initialized    ( false ),
   width          ( 1024 ), 
   height         ( 1024 ),
   display_mode   ( GLUT_DOUBLE | GLUT_RGBA | GLUT_DEPTH ),
   title          ( string("PBA Viewer") ),
   mouse_x        ( 0 ),
   mouse_y        ( 0 ),
   camera_fov     (85.0), // 120, 35.0
   camera_aspect  (1.0),
   camera_near    (0.01),
   camera_far     (10000.0),
   camera_eye_x   (0.0),
   camera_eye_y   (0.0),
   camera_eye_z   (-5.0), // -5.0
   camera_view_x  (0.0),
   camera_view_y  (0.0),
   camera_view_z  (0.0),
   camera_up_x    (0.0),
   camera_up_y    (1.0),
   camera_up_z    (0.0),
   camera_right_x (1.0),
   camera_right_y (0.0),
   camera_right_z (0.0),
	WorldUp			(glm::vec3(0.0f, 1.0f, 0.0f)),
	Yaw				(90.0f),
	Pitch			(0.0f),
	Front			(glm::vec3(0.0f, 0.0f, -1.0f)),
	pressTime		(0.0)

{
   cout << "PbaViewer Loaded\n";
}

PbaViewer::~PbaViewer(){}

void PbaViewer::Init( const std::vector<std::string>& args )
{
   int argc = (int)args.size();
   char** argv = new char*[argc];
   for( int i=0;i<argc;i++)
   {
      argv[i] = new char[args[i].length() + 1];
      std::strcpy(argv[i], args[i].c_str());
   }

   glutInit( &argc, argv );
   glutInitDisplayMode( display_mode );
   glutInitWindowSize( width, height );
   glutCreateWindow( title.c_str() );
   glClearColor(0.2,0.2,0.3,0.0);

   camera_aspect = (float)width/(float)height;

   glutDisplayFunc( &cbDisplayFunc );
   glutIdleFunc( &cbIdleFunc );
   glutKeyboardFunc( &cbKeyboardFunc );
   glutMotionFunc( &cbMotionFunc );
   glutPassiveMotionFunc(&cbPassiveMotionFunc);
   glutMouseFunc( &cbMouseFunc );
   glutReshapeFunc( &cbReshapeFunc );

   for( size_t i=0;i<things.size();i++)
   {
      things[i]->Init(args);
   }
   initialized = true;
   cout << "PbaViewer Initialized\n";
}

void PbaViewer::MainLoop()
{
   Usage();
   glutMainLoop();
}


void PbaViewer::Display()
{
   glLoadIdentity();
   gluPerspective( camera_fov, camera_aspect, camera_near, camera_far );
   gluLookAt(camera_eye_x, camera_eye_y, camera_eye_z,							// Camera eye point
	   camera_eye_x + Front.x, camera_eye_y + Front.y, camera_eye_z + Front.z,	// Camera view point
	   Up.x, Up.y, Up.z															// Camera up direction
   );

   glEnable(GL_DEPTH_TEST);
   glDepthRange( camera_near, camera_far );

   if (button == GLUT_LEFT_BUTTON)
   {
	   if (mouse_state == GLUT_DOWN)
	   {
		   pressTime += 0.02;
		   std::cout << " " << pressTime << std::endl;
	   }
	   else
		   pressTime = 0;
   }

   for( size_t i=0;i<things.size();i++)
   {
      if( things[i]->isVisible() ) { things[i]->Display(); }
   }
}


void PbaViewer::Reshape( int w, int h )
{
   width = w;
   height = h;
   camera_aspect = (float)width/(float)height;

   glViewport( 0, 0, (GLsizei) width, (GLsizei) height );
   glMatrixMode( GL_PROJECTION );
   glLoadIdentity();

   for( size_t i=0;i<things.size();i++)
   {
      things[i]->Reshape(w,h);
   }
}

void PbaViewer::Keyboard( unsigned char key, int x, int y )
{
	keystate = glutGetModifiers();

   switch (key)
   {
		case 'w':ProcessKeyboard(FORWARD, 0.1f);
			break;
		case 's':ProcessKeyboard(BACKWARD, 0.1f);
			break;
		case 'a':ProcessKeyboard(LEFT, 0.1f);
			break;
		case 'd':ProcessKeyboard(RIGHT, 0.1f);
			break;
	 case 'f':
         camera_fov /= 1.01;
         break;
      case 'F':
         camera_fov *= 1.01;
         if( camera_fov > 170.0){ camera_fov = 170.0; }
	 break;
      case '+':
      case '=':
		  if (keystate == GLUT_ACTIVE_SHIFT)
			  ComputeEyeShift(7.0);
         ComputeEyeShift(0.07);
         break;
      case '-':
      case '_':
		  if (keystate == GLUT_ACTIVE_SHIFT)
			  ComputeEyeShift(-7.0);
         ComputeEyeShift(-0.07);
         break;
      case 'r':
	Reset();
        break;
      case 'h':
	Home();
      break;
   }

   for( size_t i=0;i<things.size();i++)
   {
      things[i]->Keyboard(key,x,y);
   }
}


void PbaViewer::Motion( int x, int y )
{
   float dx = x - mouse_x;
   float dy = y - mouse_y;
   float pos_x = current_raster_pos[0] + dx;
   float pos_y = current_raster_pos[1] - dy;
   glRasterPos2f( pos_x, pos_y ); 

   // camera motion perp to view direction
   if(keystate == GLUT_ACTIVE_SHIFT )
   {
     ComputeEyeUpRight(dx,dy);
   }
   //else if (keystate == GLUT_ACTIVE_CTRL)
   //{
   //}
   //else if (keystate == GLUT_ACTIVE_ALT)
   //{
   //}

   for( size_t i=0;i<things.size();i++)
   {
      things[i]->Motion(x,y);
   }
   mouse_x = x;
   mouse_y = y;
}


void PbaViewer::PassiveMotion(int x, int y)
{
	if (firstMouse)
	{
		mouse_x = x;
		mouse_y = y;
		firstMouse = false;
	}

	float xoffset = x - mouse_x;
	float yoffset = y - mouse_y; // reversed since y-coordinates go from bottom to top

	mouse_x = x;
	mouse_y = y;

	ProcessMouseMovement(xoffset, -yoffset);
}

void PbaViewer::Mouse( int b, int state, int x, int y )
{
	float dx = x - mouse_x;
	float dy = y - mouse_y;
	float pos_x = current_raster_pos[0] + dx;
	float pos_y = current_raster_pos[1] - dy;
	glRasterPos2f(pos_x, pos_y);

   mouse_x = x;
   mouse_y = y;
   keystate = glutGetModifiers();
   button = b;
   mouse_state = state;
   glGetFloatv( GL_CURRENT_RASTER_POSITION, current_raster_pos );

   for( size_t i=0;i<things.size();i++)
   {
      things[i]->Mouse(b,state,x,y);
   }
}

void PbaViewer::Idle()
{
   for( size_t i=0;i<things.size();i++)
   {
      things[i]->Idle();
   }
}


void PbaViewer::Usage()
{
   cout << "PBA Viewer   usage:\n";
   cout << "f/F          reduce/increase the camera FOV\n";
   cout << "+/=          move camera farther from the view point\n";
   cout << "-/_          move camera closer to the view point\n";
   cout << "SHIFT+mouse  move camera perpendicular to the view direction\n";
   cout << "r            reset sim parameters\n";
   cout << "h            home display parameters\n";
 
   for( size_t i=0;i<things.size();i++)
   {
      things[i]->Usage();
   }
}

void PbaViewer::sendMessage(unsigned int _messgae)
{
	for (size_t i = 0; i < things.size(); i++)
	{
		things[i]->messageEvent(_messgae);
	}
}

void PbaViewer::Reset()
{
   std::cout << "Reset\n";
   for( size_t i=0;i<things.size();i++)
   {
      things[i]->Reset();
   }
}

void PbaViewer::Home()
{
//    std::cout << "Home\n";
//    camera_fov     = 120.0;
//    camera_near    = 0.01;
//    camera_far     = 100.0;
   camera_eye_x   = 0.0;
   camera_eye_y   = 0.0;
   camera_eye_z   = -5.0;
//    camera_view_x  = 0.0;
//    camera_view_y  = 0.0;
//    camera_view_z  = 0.0;
//    camera_up_x    = 0.0;
//    camera_up_y    = 1.0;
//    camera_up_z    = 0.0;
//    camera_right_x = 1.0;
//    camera_right_y = 0.0;
//    camera_right_z = 0.0;

   WorldUp = (glm::vec3(0.0f, 1.0f, 0.0f));
   Yaw = (90.0f);
   Pitch = (0.0f);
   Front = (glm::vec3(0.0f, 0.0f, -1.0f));
   pressTime = (0.0);
 
   for( size_t i=0;i<things.size();i++)
   {
      things[i]->Home();
   }
}



void PbaViewer::ComputeEyeUpRight(int dx, int dy)
{

// dx --> rotation around y axis
// dy --> rotation about camera right axis


   float vvx = camera_eye_x-camera_view_x; 
   float vvy = camera_eye_y-camera_view_y; 
   float vvz = camera_eye_z-camera_view_z;
   float vvnorm = std::sqrt( vvx*vvx + vvy*vvy + vvz*vvz );
   vvx /= vvnorm;
   vvy /= vvnorm;
   vvz /= vvnorm;


// Rotate around y axis
//      Rotate view direction
   float cosx = std::cos( -dx*0.006 );
   float sinx = std::sin( -dx*0.006 );
   float nvvx = vvx*cosx + vvz*sinx;
   float nvvz = -vvx*sinx + vvz*cosx;
   float nrightx = camera_right_x*cosx + camera_right_z*sinx;
   float nrightz = -camera_right_x*sinx + camera_right_z*cosx;
   vvx = nvvx;
   vvz = nvvz;
   camera_right_x = nrightx;
   camera_right_z = nrightz;
//      Rotate up direction
   float crossx = camera_up_z;
   float crossy = 0.0;
   float crossz = -camera_up_x;
   float ydotup = camera_up_y;
   camera_up_x = camera_up_x*cosx + crossx*sinx;
   camera_up_y = camera_up_y*cosx + ydotup*(1.0-cosx) + crossy*sinx;
   camera_up_z = camera_up_z*cosx + crossz*sinx;
//      Rotate right direction
   crossx = camera_right_z;
   crossy = 0.0;
   crossz = -camera_right_x;
   ydotup = camera_right_y;
   camera_right_x = camera_right_x*cosx + crossx*sinx;
   camera_right_y = camera_right_y*cosx + ydotup*(1.0-cosx) + crossy*sinx;
   camera_right_z = camera_right_z*cosx + crossz*sinx;


// Rotate around camera-right axis
//     Rotate view direction
   cosx = std::cos( dy*0.006 );
   sinx = std::sin( dy*0.006 );
   float rightdotview = camera_right_x*vvx + camera_right_y*vvy + camera_right_z*vvz;
   crossx = camera_right_y*vvz - camera_right_z*vvy;
   crossy = camera_right_z*vvx - camera_right_x*vvz;
   crossz = camera_right_x*vvy - camera_right_y*vvx;
   nvvx = vvx*cosx + camera_right_x*rightdotview*(1.0-cosx) + crossx*sinx; 
   float nvvy = vvy*cosx + camera_right_y*rightdotview*(1.0-cosx) + crossy*sinx; 
   nvvz = vvz*cosx + camera_right_z*rightdotview*(1.0-cosx) + crossz*sinx; 
   vvx = nvvx;
   vvy = nvvy;
   vvz = nvvz;
//      Rotate up direction
   crossx = camera_right_y*camera_up_z - camera_right_z*camera_up_y;
   crossy = camera_right_z*camera_up_x - camera_right_x*camera_up_z;
   crossz = camera_right_x*camera_up_y - camera_right_y*camera_up_x;
   camera_up_x = camera_up_x*cosx + crossx*sinx; 
   camera_up_y = camera_up_y*cosx + crossy*sinx; 
   camera_up_z = camera_up_z*cosx + crossz*sinx; 


   camera_eye_x = vvx*vvnorm + camera_view_x;
   camera_eye_y = vvy*vvnorm + camera_view_y;
   camera_eye_z = vvz*vvnorm + camera_view_z;
}

void PbaViewer::ComputeEyeShift(float dz)
{
   float vvx = camera_eye_x-camera_view_x; 
   float vvy = camera_eye_y-camera_view_y; 
   float vvz = camera_eye_z-camera_view_z;
   float vvnorm = std::sqrt( vvx*vvx + vvy*vvy + vvz*vvz );
   vvx /= vvnorm;
   vvy /= vvnorm;
   vvz /= vvnorm;

   camera_eye_x += dz*vvx;
   camera_eye_y += dz*vvy;
   camera_eye_z += dz*vvz;
}

void PbaViewer::ProcessKeyboard(Camera_Movement direction, float dt)
{
	float velocity = 1.0f * dt;
	if (direction == FORWARD) {
		camera_eye_x += (Front * velocity).x;
		camera_eye_y += (Front * velocity).y;
		camera_eye_z += (Front * velocity).z;
	}
	if (direction == BACKWARD) {
		camera_eye_x -= (Front * velocity).x;
		camera_eye_y -= (Front * velocity).y;
		camera_eye_z -= (Front * velocity).z;
	}
	if (direction == LEFT) {
		camera_eye_x -= (Right * velocity).x;
		camera_eye_y -= (Right * velocity).y;
		camera_eye_z -= (Right * velocity).z;
	}
	if (direction == RIGHT) {
		camera_eye_x += (Right * velocity).x;
		camera_eye_y += (Right * velocity).y;
		camera_eye_z += (Right * velocity).z;
	}
}

void PbaViewer::ProcessMouseMovement(float xoffset, float yoffset, bool constrainPitch /*= true*/)
{
	xoffset *= .8f/*0.1f*/;
	yoffset *= .8f/*0.1f*/;

	Yaw += xoffset;
	Pitch += yoffset;

	// Make sure that when pitch is out of bounds, screen doesn't get flipped
	if (constrainPitch)
	{
		if (Pitch > 89.0f)
			Pitch = 89.0f;
		if (Pitch < -89.0f)
			Pitch = -89.0f;
	}

	// Update Front, Right and Up Vectors using the updated Euler angles
	updateCameraVectors();
}

void PbaViewer::updateCameraVectors()
{
	// Calculate the new Front vector
	glm::vec3 front;
	front.x = cos(glm::radians(Yaw)) * cos(glm::radians(Pitch));
	front.y = sin(glm::radians(Pitch));
	front.z = sin(glm::radians(Yaw)) * cos(glm::radians(Pitch));
	Front = glm::normalize(front);
	// Also re-calculate the Right and Up vector
	Right = glm::normalize(glm::cross(Front, WorldUp));  // Normalize the vectors, because their length gets closer to 0 the more you look up or down which results in slower movement.
	Up = glm::normalize(glm::cross(Right, Front));
}

void PbaViewer::AddThing( pba::PbaThing& t ) 
{ 
   things.push_back(t); 
   std::cout << t->Name() << " added to viewer.\n";
}




PbaViewer* CreateViewer() { return PbaViewer::Instance(); }



}


