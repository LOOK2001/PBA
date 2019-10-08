#include "AsciiParser.h"

pba::AsciiParser::AsciiParser()
{
}

const bool pba::AsciiParser::ParseFile(const string& path)
{
	FILE* file = fopen(path.c_str(), "r");
	if (file == NULL) {
		printf("Impossible to open the file !n");
		return false;
	}

	return false;
}
