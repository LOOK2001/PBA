#ifndef ____PBA_PLYPARSER_H____
#define ____PBA_PLYPARSER_H____

#include <string>
using namespace std;
#include "AsciiParser.h"
#include "CollisionSurface.h"
#include "DynamicalState.h"
//#include "SoftBodyState.h"

namespace pba
{
	class ObjParser
	{
	public:

		ObjParser() {}
		~ObjParser() {}

		const bool ParseFile(const string& filename);
		const bool Fill(CollisionSurface& g);
		const bool Fill(DynamicalState& g);
		const bool Fill(RigidBodyState& g);
		//const bool Fill(SoftBodyState& g);
		const bool List();

		const bool IsObjLine() const;
		const bool IsComment() const;
		const bool IsTexture() const;
		const bool IsVertex() const;
		const bool IsFace() const;
		const bool IsGroup() const;

		const bool AdvanceToNextLine();
		const bool GetVertex(double& x, double& y, double& z);
		const bool GetTextureCoordinate(double& x, double& y, double& z);
		const bool GetFace(int& x, int& y, int& z);
		const bool GetTexturedFace(int& x, int& y, int& z, int& xt, int& yt, int& zt);

		const bool GetObjLine();
		const int  GetVertexCount();
		const int  GetFaceCount();
		const bool GetHeaderEnd() { return false; }

	private:

		AsciiParser parser;
		int nb_vertices;
		int nb_faces;
		meshData mesh;
	};
}

#endif