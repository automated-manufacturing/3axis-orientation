#include "MachiningOrientation.h"

void macho::readStl(std::string inputFilename, vtkSmartPointer<vtkPolyData> meshModel)
{
	vtkNew<vtkSTLReader> reader;
	reader->SetFileName(inputFilename.c_str());
	reader->Update();

	meshModel->ShallowCopy(reader->GetOutput());
}

// Catch mouse events
class MouseInteractorStyle : public vtkInteractorStyleTrackballCamera
{
public:
	static MouseInteractorStyle* New();

	// override the right button click function to select facet on model
	// right button usually controls zoom, but I'm disabling that functionality
	// just zoom with the wheel instead

	virtual void OnRightButtonDown() override
	{
		// Get the location of the click (in window coordinates)
		int* pos = this->GetInteractor()->GetEventPosition();
		//std::cout << "Selection position: " << pos[0] << ", " << pos[1] << std::endl;

		// Pick from this location.
		vtkNew<vtkCellPicker> picker;
		picker->SetTolerance(0.0005);
		picker->Pick(pos[0], pos[1], 0, this->GetDefaultRenderer());

		//std::cout << "Cell id is: " << picker->GetCellId() << std::endl;
		this->selectedCellIndex = picker->GetCellId();

		if (picker->GetCellId() != -1)
		{
			this->thisWindow->Finalize();
			this->thisInteractor->TerminateApp();
		}
	}

	int selectedCellIndex;
	vtkSmartPointer<vtkRenderWindow> thisWindow;
	vtkSmartPointer<vtkRenderWindowInteractor> thisInteractor;
};

vtkStandardNewMacro(MouseInteractorStyle);

void macho::pick3axisMachiningDirection(vtkSmartPointer<vtkPolyData> meshModel)
{
	/*
	This function allows the user to click a face on the model mesh to define the machining orientation
	The model will appear in a new VTK window, selection is made upon right clicking a face on the model

	If function isn't behaving as expected, modification of the "angleVariability" parameter may be useful
	*/

	vtkNew<vtkNamedColors> colors;

	vtkNew<vtkPolyDataMapper> mapper;
	mapper->SetInputData(meshModel);

	vtkNew<vtkActor> actor;
	actor->GetProperty()->SetColor(colors->GetColor3d("Coral").GetData());
	actor->SetMapper(mapper);

	vtkNew<vtkRenderer> renderer;
	vtkNew<vtkRenderWindow> renderWindow;
	renderWindow->AddRenderer(renderer);
	renderWindow->SetWindowName("CellPicking");

	vtkNew<vtkRenderWindowInteractor> renderWindowInteractor;
	renderWindowInteractor->SetRenderWindow(renderWindow);
	renderWindowInteractor->Initialize();

	// Set the custom stype to use for interaction.
	vtkNew<MouseInteractorStyle> style;
	style->SetDefaultRenderer(renderer);
	style->thisWindow = renderWindow;
	style->thisInteractor = renderWindowInteractor;

	renderWindowInteractor->SetInteractorStyle(style);

	renderer->AddActor(actor);
	renderer->ResetCamera();
	renderer->GetActiveCamera()->SetParallelProjection(true);

	renderer->SetBackground(colors->GetColor3d("PowderBlue").GetData());

	renderWindow->Render();
	renderWindowInteractor->Start();

	/*
	A cell has been selected which we have manually identified as lying on one of the faces normal to the machining direction
	We can assume this cell has normal has some noise, but is relatively close to the true machining direction
	To reduce the noise, we want to look at all of the other cells which have similar normal (assuming they should also be normal to machining direction)
	then take the average direction of all of these normals
	At this point the variability in the normals is not studied, so an arbitrary value is chosen which hopefully selects almost all of the cells we want and almost none of those we don't want
	*/

	const double PI = 4 * atan(1.);
	// define how much angle variability we want to allow in order to capture the desired cells
	double angleVariability = 10 * (PI / 180); // radians

	// calculate normals for all of the cells (weighted by area of triangular facet)
	// this could probably be done using VTK functions with better time performance, but it was easier to get the normal vectors weighted by facet size by programming this way
	std::vector<Eigen::Vector3d> weightedNormal(meshModel->GetNumberOfCells());
	Eigen::Vector3d p0, p1, p2, v01, v02;
	for (int it = 0; it < meshModel->GetNumberOfCells(); ++it)
	{
		memcpy(p0.data(), meshModel->GetCell(it)->GetPoints()->GetPoint(0), sizeof(double) * 3);
		memcpy(p1.data(), meshModel->GetCell(it)->GetPoints()->GetPoint(1), sizeof(double) * 3);
		memcpy(p2.data(), meshModel->GetCell(it)->GetPoints()->GetPoint(2), sizeof(double) * 3);
		v01 = p1 - p0;
		v02 = p2 - p0;
		weightedNormal[it] = v01.cross(v02);
	}
	// find which cells have normal vector close enough to that of chosen cell
	std::vector<bool> matchingAngle(meshModel->GetNumberOfCells(), false);
	Eigen::Vector3d n0 = weightedNormal[style->selectedCellIndex].normalized();
	Eigen::Vector3d n = n0;
	Eigen::Vector3d machiningAngle(0, 0, 0);

	// the angle averaging process is iterated a few times just in case the initial selection is a bit of a wonky facet
	// in this case it is possible that if the angleVariability parameter is small enough and the selected facet is wonky enough, then only a subset of facets which are almost the correct normal but biased in the same wonky direction will be averaged
	// this would cause the machining direction to be biased to the wonk undesirably
	// the averaging process should tend to dewonk relative to the selected normal vector, so if the averaging is iterated a few times it may help mitigate wonk problems
	for (int jt = 0; jt < 3; ++jt)
	{
		double ang;
		machiningAngle = Eigen::Vector3d(0, 0, 0);
		// calculate machining angle as weighted average of all normals which are in angle range of selected normal
		for (int it = 0; it < meshModel->GetNumberOfCells(); ++it)
		{
			ang = acos(n.dot(weightedNormal[it].normalized()));
			if (ang > PI / 2)
			{
				ang = PI - ang;
				weightedNormal[it] *= -1;
			}
			if (ang < angleVariability)
			{
				matchingAngle[it] = true;
				machiningAngle += weightedNormal[it];
			}
		}
		machiningAngle.normalize();
		n = machiningAngle;
	}

	/*
	// visualization stuff to check that parameter selection is good
	{
		// create cell color data
		vtkNew<vtkLookupTable> colorLUT;
		colorLUT->SetNumberOfTableValues(2);
		colorLUT->Build();
		colorLUT->SetTableValue(0, 1.0, 0.5, 0.5);
		colorLUT->SetTableValue(1, 0.5, 1.0, 0.5);
		vtkNew<vtkFloatArray> cellData;
		cellData->SetNumberOfValues(meshModel->GetNumberOfCells());
		for (int i = 0; i < meshModel->GetNumberOfCells(); ++i)
		{
			if (matchingAngle[i])
			{
				cellData->SetValue(i, 0);
			}
			else
			{
				cellData->SetValue(i, 1);
			}

		}
		meshModel->GetCellData()->SetScalars(cellData);

		vtkNew<vtkPolyDataMapper> mapper2;
		mapper2->SetInputData(meshModel);
		mapper2->SetScalarRange(0, 1);
		mapper2->SetLookupTable(colorLUT);

		vtkNew<vtkActor> actor2;
		actor2->SetMapper(mapper2);

		vtkNew<vtkRenderer> renderer2;
		vtkNew<vtkRenderWindow> renderWindow2;
		renderWindow2->AddRenderer(renderer2);
		renderWindow2->SetWindowName("CellPicking");

		vtkNew<vtkRenderWindowInteractor> renderWindowInteractor2;
		renderWindowInteractor2->SetRenderWindow(renderWindow2);
		renderWindowInteractor2->Initialize();

		renderer2->AddActor(actor2);
		renderer2->ResetCamera();
		renderer2->GetActiveCamera()->SetParallelProjection(true);

		renderer2->SetBackground(colors->GetColor3d("PowderBlue").GetData());

		renderWindow2->Render();
		renderWindowInteractor2->Start();
	}
	*/

	std::cout << "selected cell orientation: <" << n0.x() << ", " << n0.y() << ", " << n0.z() << ">\n";
	std::cout << "machining orientation:     <" << machiningAngle.x() << ", " << machiningAngle.y() << ", " << machiningAngle.z() << ">\n";

	macho::orientModelToMachiningDirection(meshModel, machiningAngle);

	return;
}

struct Sphere
{
	Eigen::Vector3d center;
	double radius;
};

Sphere miniballGetSeb(vtkSmartPointer<vtkPolyData> meshModel)
{
	typedef double FT;
	typedef Seb::Point<FT> Point;
	typedef std::vector<Point> PointVector;
	typedef Seb::Smallest_enclosing_ball<FT> Miniball;

	const int d = 3;

	// reformat points from meshModel into miniball-recognized format
	PointVector S(meshModel->GetNumberOfPoints(), d);
	for (vtkIdType i = 0; i < meshModel->GetNumberOfPoints(); ++i)
	{
		meshModel->GetPoint(i, &S[i][0]);
	}
	// compute the minimum enclosing sphere
	Miniball mb(d, S);
	Sphere minSphere;
	minSphere.radius = mb.radius();
	memcpy(&minSphere.center.x(), mb.center_begin(), sizeof(FT) * 3);

	return minSphere;
}

void macho::autoFind3axisOrientation(vtkSmartPointer<vtkPolyData> meshModel)
{
	/*
	This function attempts to find 3 axis machining orientation based on a parallel ray tracing technique
	ideally, for a 3 axis part there should be an orientation where when viewing from either the top or bottom, all faces are visible and there are no undercuts
	for a given orientation, undercuts can be observed by rendering the model with surface transparency; undercut regions are rendered with greater cumulative opacity
	the best orientation is assumed to be that orientation which has minimal pixels of high cumulative opacity, find that orientation

	Parameters which may be edited to affect performance
	pCount is the number of particles in the optimization. Higher pCount decreases odds of landing in a suboptimal local minima at the cost of execution speed
	windowSize is the square width of the image which is rendered and analyzed for opacity. Window size should be large enough that all features are rendered with appropriate resolution, but computational cost increases with window size
	terminationIter is the termination condition for when the optimization ends. If a new best orientation hasn't been seen for this many iterations, the optimization ends
	*/

	/*
	find the minimum enclosing sphere of the mesh.
	we will be rendering the mesh from a variety of different angles.
	the enclosing sphere will provide us with information that can be used to set camera position and orientation,
	near and far rendering planes, and "zoom" level of parallel projection.
	this allows us to guarantee that the entire part will be included at consistent size for renders from every angle
	*/
	Sphere encSphere = miniballGetSeb(meshModel);
	// add a bit of a buffer to the enclosing sphere radius just to make certain that the mesh doesn't get clipped during render
	encSphere.radius += 1;

	/*
	use modified/simplified particle swarm optimization to find orientation which minimizes overhangs
	*/
	// randomly generate particles, uniformly distributed on surface of unit sphere
	srand((unsigned int)time(0));
	int pCount = 10;
	std::vector<Eigen::Vector3d> p(pCount);
	for (int it = 0; it < pCount; it++)
	{
		p[it] = Eigen::Vector3d::Random();
		double x = p[it].x();
		double y = p[it].y();
		double z = p[it].z();
		if (y < 0)
		{
			y = -cos(EIGEN_PI * (y + 1) / 2);
		}
		else
		{
			y = cos(EIGEN_PI * (1 - (y + 1) / 2));
		}
		y *= sqrt((1 + x) * (1 - x));
		if (z < 0)
		{
			z = -sqrt(1 - x * x - y * y);
		}
		else
		{
			z = sqrt(1 - x * x - y * y);
		}
		p[it].x() = x;
		p[it].y() = y;
		p[it].z() = z;
	}
	// initialize other stuff needed for PSO
	Eigen::Vector3d posBest = Eigen::Vector3d::Zero();
	double scoreBest = 2; // score is percentage of part-intersecting rays that pass through too many faces, 1 is worst possible score

	// Render scene settings
	int windowSize = 900;
	double backgroundColor[3] = { 0, 0, 0 };
	vtkNew<vtkPolyDataMapper> mapper;
	mapper->SetInputData(meshModel);
	vtkNew<vtkActor> actor;
	actor->SetMapper(mapper);
	actor->GetProperty()->SetAmbientColor(1, 1, 1);
	actor->GetProperty()->SetDiffuseColor(0, 0, 0);
	actor->GetProperty()->SetSpecularColor(0, 0, 0);
	actor->GetProperty()->SetSpecular(0);
	actor->GetProperty()->SetDiffuse(0);
	actor->GetProperty()->SetAmbient(1);
	actor->GetProperty()->SetSpecularPower(0);
	actor->GetProperty()->SetOpacity(0.5);
	actor->GetProperty()->LightingOff();
	vtkNew<vtkRenderer> renderer;
	renderer->AddActor(actor);
	renderer->SetBackground(backgroundColor);
	renderer->GetActiveCamera()->ParallelProjectionOn();
	renderer->GetActiveCamera()->SetFocalPoint(encSphere.center.x(), encSphere.center.y(), encSphere.center.z());
	renderer->GetActiveCamera()->SetClippingRange(1, encSphere.radius * 2 - 2);
	renderer->GetActiveCamera()->SetParallelScale(encSphere.radius);
	vtkNew<vtkRenderWindow> renderWindow;
	renderWindow->SetSize(windowSize, windowSize);
	renderWindow->AddRenderer(renderer);
	renderWindow->SetOffScreenRendering(true);
	renderWindow->Render();

	// outer loop for optimization
	double cx, cy, cz, r;
	double x, y, z, g, h;
	int terminationIter = 20;
	int iterCt = 0;
	int bestIter = 0;
	vtkNew<vtkUnsignedCharArray> vtkPtr;
	renderWindow->GetPixelData(0, 0, windowSize - 1, windowSize - 1, 1, vtkPtr);
	cv::Mat RGBImage(windowSize, windowSize, CV_8UC3, vtkPtr->Begin());
	cv::Mat grayImage(windowSize, windowSize, CV_8UC1);
	cv::Mat threshImg(windowSize, windowSize, CV_8UC1);
	double pxCountA, pxCountB, thisScore;
	while (iterCt - bestIter < terminationIter)
	{
		iterCt++;
		// render for all particles
		for (int it = 0; it < pCount; ++it)
		{
			// update camera position for this particle
			cx = encSphere.center.x();
			cy = encSphere.center.y();
			cz = encSphere.center.z();
			r = encSphere.radius;
			x = p[it].x();
			y = p[it].y();
			z = p[it].z();
			renderer->GetActiveCamera()->SetPosition(cx + r * x, cy + r * y, cz + r * z);
			g = std::copysign(1.0, z);
			h = z + g;
			renderer->GetActiveCamera()->SetViewUp(g - x * x / h, -x * y / h, -x);

			// render
			renderWindow->Render();
			// move image from GPU to CPU, into opencv image location
			// moving image to CPU is slow, it would be best to process data on GPU instead, but idk how
			renderWindow->GetPixelData(0, 0, windowSize - 1, windowSize - 1, 1, vtkPtr);
			// count the number of pixels that contain part geometry
			cv::cvtColor(RGBImage, grayImage, cv::COLOR_BGR2GRAY);
			cv::threshold(grayImage, threshImg, 1, 255, cv::THRESH_BINARY);
			pxCountA = (double)cv::countNonZero(threshImg);
			// count the number of pixels that see more than 2 part intersections
			cv::threshold(grayImage, threshImg, 192, 255, cv::THRESH_BINARY);
			pxCountB = (double)cv::countNonZero(threshImg);
			// score is percertage of part pixels which see more than 2 part intersections, lower is better
			thisScore = pxCountB / pxCountA;
			// update best orientation
			if (thisScore < scoreBest)
			{
				scoreBest = thisScore;
				posBest = p[it];
				bestIter = iterCt;
				//std::wcout << "score: " << scoreBest << " at orientation: <" << posBest.x() << ", " << posBest.y() << ", " << posBest.z() << ">\n";
				/*
				// visualization stuff to see how the optimization is going
				std::string windowName = "mywindow";
				cv::namedWindow(windowName);
				cv::imshow(windowName, grayImage);
				cv::waitKey(1);
				*/
			}

		}
		// every particle moves to be closer to the best known position
		// for now, every particle moves roughly 1/nth of the way to that best known position, but with some extra noise thrown in
		Eigen::Vector3d noise;
		Eigen::Vector3d axis;
		double angle;
		Eigen::AngleAxisd rot;
		for (int it = 0; it < pCount; ++it)
		{
			noise.setRandom();
			noise *= 0.1;
			p[it] += noise;
			p[it].normalize();
			axis = p[it].cross(posBest);
			axis.normalize();
			angle = acos(p[it].dot(posBest)) / 2;
			rot = Eigen::AngleAxisd(angle, axis);
			p[it] = rot * p[it];
		}
	}

	Eigen::Vector3d machiningAngle = posBest;
	std::cout << "machining orientation:     <" << machiningAngle.x() << ", " << machiningAngle.y() << ", " << machiningAngle.z() << ">\n";

	macho::orientModelToMachiningDirection(meshModel, machiningAngle);

	return;
}

void macho::orientModelToMachiningDirection(vtkSmartPointer<vtkPolyData> meshModel, Eigen::Vector3d machiningAngle)
{
	/*
	Rotate the mesh so that the machining direction aligns with the positive z axis
	*/

	// incoming machining orientation should already be normalized, but normalize it here just in case
	machiningAngle.normalize();
	// calculate rotation axis as the cross product of the machining direction and positive z axis
	Eigen::Vector3d rotAx = machiningAngle.cross(Eigen::Vector3d(0, 0, 1));
	rotAx.normalize();
	// calculate the angle between these two vectors
	double rotAng = acos(machiningAngle.z());
	const double PI = 4 * atan(1.);
	rotAng *= 180 / PI;

	// apply rotation to the model
	vtkNew<vtkTransform> transform;
	transform->RotateWXYZ(rotAng, rotAx.x(), rotAx.y(), rotAx.z());
	transform->Update();
	vtkNew<vtkTransformPolyDataFilter> transformFilter;
	transformFilter->SetInputData(meshModel);
	transformFilter->SetTransform(transform);
	transformFilter->Update();
	meshModel->DeepCopy(transformFilter->GetOutput());

	return;
}
