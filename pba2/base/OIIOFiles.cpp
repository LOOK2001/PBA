
#include "OIIOFiles.h"

#include <OpenImageIO/imageio.h> 

#include <sys/types.h>
#include <sys/stat.h>
//#include <unistd.h>
#include <io.h>

#include <iostream>

using namespace std;
using namespace pba;
OIIO_NAMESPACE_USING


void pba::writeOIIOImage(std::string& filename, float* imagedata, int nx, int ny, const std::map<std::string,std::string>& metadata)
{
   std::unique_ptr<ImageOutput> out = ImageOutput::create (filename.c_str()); 
   if( !out )
   {
      cout << "Not able to write an image to file " << filename << endl;
   }
   else
   {
      ImageSpec spec (nx, ny, 3, TypeDesc::FLOAT); 
      spec.attribute("user", "pba");
      spec.attribute("writer", "OIIOFiles" );

      std::map<std::string,std::string>::const_iterator md = metadata.begin();
	  while( md != metadata.end() )
	  {
	     const string& name = md->first;
	     const string& value = md->second;
	     spec.attribute( name, value );
	     md++;
	  }

      out->open (filename.c_str(), spec);
      out->write_image (TypeDesc::FLOAT, imagedata); 
      out->close (); 
   }
}

