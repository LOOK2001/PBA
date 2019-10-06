

#ifndef __PBA_OIIOFILES_H__
#define __PBA_OIIOFILES_H__



#include <string>
#include <map>

namespace pba{


void writeOIIOImage(std::string& f, float* img, int nx, int ny, const std::map<std::string,std::string>& metadata);

}

#endif
