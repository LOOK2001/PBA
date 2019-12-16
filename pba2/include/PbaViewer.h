//-------------------------------------------------------
//
//  PbaViewer.h
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
//
//  Copyright (c) 2003,2017 Jerry Tessendorf
//
//
//--------------------------------------------------------


#ifndef ____PBA_VIEWER_H____
#define ____PBA_VIEWER_H____

#include <cstring>
#include <vector>
#include "Vector.h"
#include "PbaThing.h"

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

using namespace std;

namespace pba{

	enum Camera_Movement {
		FORWARD,
		BACKWARD,
		LEFT,
		RIGHT
	};


class PbaViewer
{
  public:

    //! The viewer is a singleton
    static  PbaViewer* Instance()
    {
       if(pPbaViewer==nullptr)
       {
          pPbaViewer = new PbaViewer();
       }
       return pPbaViewer;
    }

    ~PbaViewer();

    //! Initialization, including GLUT initialization.
    void Init( const std::vector<std::string>& args );
    //! Invokes the GLUT main loop.
    void MainLoop();
    
    //! Set the window width
    void SetWidth( const int w ) { width = w; }
    //! Set the window height 
    void SetHeight( const int h ) { height = h; }

    //! Get the window width
    const int& GetWidth() { return width;  }
    //! Get the window height 
    const int& GetHeight() { return height; }

    //! Set the window title
    void SetTitle( const std::string& t ){ title = t; }
    //! Set the window title
    void SetTitle( const char * t ) { title = t; }
    //! Get the window title
    const std::string& GetTitle() { return title; }

    // Callback functions
    //! Cascading callback for initiating a display event
    void Display();
    //! Cascading callback for a keyboard event 
    void Keyboard( unsigned char key, int x, int y );
    //! Cascading callback for a mouse event 
    void Mouse( int button, int state, int x, int y );
    //! Cascading callback for a mouse motion event 
    void Motion( int x, int y );
	void PassiveMotion(int x, int y);
    //! Cascading callback for a GLUT Special event 
    void Special( int key, int x, int y ){}
    //! Cascading callback for an idle  event 
    void Idle();
    //! Cascading callback for a window reshape 
    void Reshape( int w, int h );
    //! Cascading callback for reseting parameters
    void Reset();
    //! Cascading callback to home parameters
    void Home();

    void AddThing( pba::PbaThing& t );

    //! Cascading callback for usage information
    void Usage();

	void sendMessage(unsigned int _messgae);
	Vector getCamerPos() { return Vector(camera_eye_x, camera_eye_y, camera_eye_z); }
	Vector getCamerDir() { return Vector(camera_eye_x + Front.x - camera_eye_x, 
								camera_eye_y + Front.y - camera_eye_y, 
								camera_eye_z + Front.z - camera_eye_z).unitvector(); }
	double getPressTime() {
		if (pressTime < 1.0) return 1.0;
		else return pressTime;
	}

  private:

    bool initialized;
    int width, height;
    unsigned int display_mode;

    std::string title;
    int mouse_x, mouse_y;
    int keystate, button;
    int mouse_state;
    float current_raster_pos[4];
   
    float camera_fov;
    float camera_aspect;
    float camera_near;
    float camera_far;
    float camera_eye_x, camera_eye_y, camera_eye_z;
    float camera_view_x, camera_view_y, camera_view_z;
    float camera_up_x, camera_up_y, camera_up_z;
    float camera_right_x, camera_right_y, camera_right_z;

	// Euler Angles
	float Yaw;
	float Pitch;
	glm::vec3 Front;
	glm::vec3 Up;
	glm::vec3 Right;
	glm::vec3 WorldUp;
	bool firstMouse = true;
	double pressTime;

    void ComputeEyeUpRight(int dx, int dy);
    void ComputeEyeShift(float dz);
	void ProcessKeyboard(Camera_Movement direction, float dt);
	void ProcessMouseMovement(float xoffset, float yoffset, bool constrainPitch = true);
	void updateCameraVectors();

    // These are the objects that do the important processing. 
    std::vector<pba::PbaThing> things;
 
    static PbaViewer* pPbaViewer;

    // dont allow any of these
    PbaViewer();
    PbaViewer( const PbaViewer& );
    PbaViewer& operator= (const PbaViewer&);

};


PbaViewer* CreateViewer();



}





#endif
