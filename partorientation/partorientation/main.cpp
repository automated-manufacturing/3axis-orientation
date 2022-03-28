#include "MachiningOrientation.h"

int main()
{

	std::string stlFile = "C:\\Users\\Andrew Dahl\\Desktop\\impeller.stl";
	vtkNew<vtkPolyData> meshModel;
	macho::readStl(stlFile, meshModel);

	macho::pick3axisMachiningDirection(meshModel);
	//macho::autoFind3axisOrientation(meshModel);

	macho::fixtureOrientByBoundingBox(meshModel);

	vtkNew<vtkSTLWriter> stlWriter;
	stlWriter->SetFileName("C:\\Users\\Andrew Dahl\\Desktop\\impeller_rot.stl");
	stlWriter->SetInputData(meshModel);
	stlWriter->SetFileTypeToBinary();
	stlWriter->Write();

	return 0;
}
