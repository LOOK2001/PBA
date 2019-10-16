#include "AsciiParser.h"
#include <fstream>
#include <math.h>

pba::AsciiParser::AsciiParser()
{
}

const bool pba::AsciiParser::ParseFile(const string& path, meshData& mesh)
{
	if (path.substr(path.size() - 4, 4) != ".obj")
		return false;

	std::ifstream file(path);
	if (!file.is_open())
		return false;

	std::string curline;
	while (std::getline(file, curline))
	{
		// Generate a Vertex Position
		if (algorithm::firstToken(curline) == "v")
		{
			std::vector<std::string> spos;
			Vector vpos;
			algorithm::split(algorithm::tail(curline), spos, " ");

			vpos[0] = std::stof(spos[0]);
			vpos[1] = std::stof(spos[1]);
			vpos[2] = std::stof(spos[2]);

			mesh.Positions.push_back(vpos);
		}

		// Generate a Vertex Texture Coordinate
		if (algorithm::firstToken(curline) == "vt")
		{
			std::vector<std::string> stex;
			float vtex[2];
			algorithm::split(algorithm::tail(curline), stex, " ");

			vtex[0] = std::stof(stex[0]);
			vtex[1] = std::stof(stex[1]);

			//TCoords.push_back(vtex);
		}

		// Generate a Vertex Normal;
		if (algorithm::firstToken(curline) == "vn")
		{
			std::vector<std::string> snor;
			Vector vnor;
			algorithm::split(algorithm::tail(curline), snor, " ");

			vnor[0] = std::stof(snor[0]);
			vnor[1] = std::stof(snor[1]);
			vnor[2] = std::stof(snor[2]);

			mesh.Normals.push_back(vnor);
		}

		// Generate a Face (vertices & indices)
		if (algorithm::firstToken(curline) == "f")
		{
			std::vector<std::string> snor;
			std::vector<std::string> snor2;
			algorithm::split(algorithm::tail(curline), snor, " ");

			for (int j = 0; j < 3; j++)
			{
				algorithm::split(snor[j], snor2, "//");
				mesh.Indices.push_back((std::stoi(snor2[0])-1));
			}

			unsigned int test0, test1, test2;
			test0 = mesh.Indices[mesh.Indices.size() - 3];
			test1 = mesh.Indices[mesh.Indices.size() - 2];
			test2 = mesh.Indices[mesh.Indices.size() - 1];

			if ((test0 == test1) || (test0 == test2) || (test1 == test2))
			{
				int success = 123;
			}
		}
	}
	return true;
}

void pba::AsciiParser::AddToken(const string& line)
{
	tokens.push_back(line);
}
