#include "MachiningOrientation.h"
#include <vtkSTLWriter.h>

int main()
{

	std::string stlFile = "C:\\Users\\Andrew Dahl\\Desktop\\impeller.stl";
	vtkNew<vtkPolyData> meshModel;
	macho::readStl(stlFile, meshModel);

	macho::pick3axisMachiningDirection(meshModel);
	//macho::autoFind3axisOrientation(meshModel);

	return 0;
}