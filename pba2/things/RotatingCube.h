//-------------------------------------------------------
//
//  RotatingCube.C
//
//  PbaThing for RotatingCube
//
//  Copyright (c) 2017 Jerry Tessendorf
//
//
//--------------------------------------------------------


#include "PbaThing.h"
#include "Vector.h"
#include "Color.h"

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

namespace pba{


class RotatingCubeThing: public PbaThingyDingy
{
  public:

    RotatingCubeThing(const std::string nam="RotatingCubeThing") :
       PbaThingyDingy (nam),
       axis    (pba::Vector(0,1,0)),
       theta   (0.0),
       dtheta  (24.0*3.14159265/180.0)
      {
         std::cout << name << " constructed\n";
      };
   ~RotatingCubeThing(){};

    //! Initialization, including GLUT initialization.
    void Init( const std::vector<std::string>& args )
    {
       verts[0] = pba::Vector(-1,-1,-1);
       verts[1] = pba::Vector(1,-1,-1);
       verts[2] = pba::Vector(1,1,-1);
       verts[3] = pba::Vector(-1,1,-1);
       verts[4] = pba::Vector(-1,-1,1);
       verts[5] = pba::Vector(1,-1,1);
       verts[6] = pba::Vector(1,1,1);
       verts[7] = pba::Vector(-1,1,1);

       normals[0] = pba::Vector(1,0,0);
       normals[1] = pba::Vector(0,1,0);
       normals[2] = pba::Vector(0,0,1);
       normals[3] = pba::Vector(-1,0,0);
       normals[4] = pba::Vector(0,-1,0);
       normals[5] = pba::Vector(0,0,-1);

       face_colors[0] = pba::Color(1,0,1,1);
       face_colors[1] = pba::Color(1,0,0,1);
       face_colors[2] = pba::Color(0,0,1,1);
       face_colors[3] = pba::Color(0,1,0,1);
       face_colors[4] = pba::Color(1,1,0,1);
       face_colors[5] = pba::Color(0,1,1,0);

       std::vector<int> face;
       face.push_back(1);
       face.push_back(2);
       face.push_back(6);
       face.push_back(5);
       faces.push_back(face);

       face[0] = 2;
       face[1] = 3;
       face[2] = 7;
       face[3] = 6;
       faces.push_back(face);

       face[0] = 0;
       face[1] = 3;
       face[2] = 2;
       face[3] = 1;
       faces.push_back(face);

       face[0] = 0;
       face[1] = 4;
       face[2] = 7;
       face[3] = 3;
       faces.push_back(face);

       face[0] = 0;
       face[1] = 1;
       face[2] = 5;
       face[3] = 4;
       faces.push_back(face);

       face[0] = 5;
       face[1] = 6;
       face[2] = 7;
       face[3] = 4;
       faces.push_back(face);
     }
    
    // Callback functions
    //! Cascading callback for initiating a display event
    void Display() 
    {
		glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA); //指定混合函数
		glEnable(GL_BLEND);

       glBegin(GL_QUADS);
       for( size_t i=0;i<faces.size();i++ )
       {
          //glColor3f( face_colors[i].red(), face_colors[i].green(), face_colors[i].blue() );
		  glColor4f(face_colors[i].red(), face_colors[i].green(), face_colors[i].blue(), face_colors[i].alpha());
          std::vector<int>& face = faces[i];

          pba::Vector v = Vertex(face[0]);
          glVertex3f( v.X(), v.Y(), v.Z() );
          v = Normal(face[0]);
          glNormal3f( v.X(), v.Y(), v.Z() );

          v = Vertex(face[1]);
          glVertex3f( v.X(), v.Y(), v.Z() );
          v = Normal(face[1]);
          glNormal3f( v.X(), v.Y(), v.Z() );

          v = Vertex(face[2]);
          glVertex3f( v.X(), v.Y(), v.Z() );
          v = Normal(face[2]);
          glNormal3f( v.X(), v.Y(), v.Z() );

          v = Vertex(face[3]);
          glVertex3f( v.X(), v.Y(), v.Z() );
          v = Normal(face[3]);
          glNormal3f( v.X(), v.Y(), v.Z() );
       }
	   glDisable(GL_BLEND);
       glEnd();
    };

    //! Cascading callback for an idle  event 
    void solve(){ theta += dtheta*dt; };
    //! Cascading callback for reseting parameters
    void Reset(){ theta = 0.0; };
    //! Cascading callback for usage information
    void Usage()
    {
       PbaThingyDingy::Usage();
    }; 

    //! Vertex after rotation
    const pba::Vector Vertex(const int i ) const
    {
       int ii = i % 8;
       pba::Vector result = verts[ii].rotate( axis, theta );
       return result;
    }

    //! Normal after rotation
    const pba::Vector Normal(const int i ) const
    {
       int ii = i % 6;
       pba::Vector result = normals[ii].rotate( axis, theta );
       return result;
    }

    

  private:

    // rotation axis
    pba::Vector axis;
    double theta;
    double dtheta;
    
    // vertices
    pba::Vector verts[8];

    // face normals
    pba::Vector normals[6];

    // faces
    std::vector< std::vector<int> > faces;

    // face colors
    pba::Color face_colors[6];

};



pba::PbaThing RotatingCube(){ return PbaThing( new pba::RotatingCubeThing() ); }



}





