#ifndef MACHORIENT
#define MACHORIENT

#include <iostream>

#include <vtkActor.h>
#include <vtkNamedColors.h>
#include <vtkNew.h>
#include <vtkPolyData.h>
#include <vtkPolyDataMapper.h>
#include <vtkProperty.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkRenderer.h>
#include <vtkSTLReader.h>
#include <vtkBoundingBox.h>
#include <vtkTriangleMeshPointNormals.h>
#include <vtkCamera.h>
#include <vtkCellData.h>
#include <vtkWindowToImageFilter.h>
#include <vtkPolyDataNormals.h>
#include <vtkFloatArray.h>
#include <vtkDecimatePro.h>
#include <vtkQuadricDecimation.h>
#include <vtkSmoothPolyDataFilter.h>
#include <vtkThreshold.h>
#include <vtkImageDataGeometryFilter.h>
#include <vtkInteractorStyleTrackballCamera.h>
#include <vtkDataSetMapper.h>
#include <vtkCellPicker.h>
#include <vtkSelectionNode.h>
#include <vtkSelection.h>
#include <vtkExtractSelection.h>
#include <vtkUnstructuredGrid.h>
#include <vtkRendererCollection.h>
#include <vtkLookupTable.h>
#include <vtkPoints.h>
#include <vtkTransform.h>
#include <vtkTransformPolyDataFilter.h>
#include <vtkSTLWriter.h>
#include <vtkOBBTree.h>

#include "vtkAutoInit.h"
VTK_MODULE_INIT(vtkRenderingOpenGL2);
VTK_MODULE_INIT(vtkInteractionStyle);

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <Eigen/Dense>

#include <Seb.h>

namespace macho
{

	void readStl(std::string inputFilename, vtkSmartPointer<vtkPolyData> meshModel);
	void pick3axisMachiningDirection(vtkSmartPointer<vtkPolyData> meshModel);
	void autoFind3axisOrientation(vtkSmartPointer<vtkPolyData> meshModel);
	void orientModelToMachiningDirection(vtkSmartPointer<vtkPolyData> meshModel, Eigen::Vector3d machiningAngle);
	void fixtureOrientByBoundingBox(vtkSmartPointer<vtkPolyData> meshModel);
	void orientModelForFixturing(vtkSmartPointer<vtkPolyData> meshModel, double zRotAngle);

}

#endif