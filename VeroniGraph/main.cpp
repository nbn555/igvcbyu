
#include <iostream>

#include <mrpt/poses/CPose3D.h>//CPose3D must be included before CMultiMetricMap
#include <mrpt/slam/CMultiMetricMap.h>
#include <mrpt/poses/CPoint3D.h>
#include <mrpt/math/CMatrixTemplateNumeric.h>
#include <mrpt/gui.h>

using namespace mrpt;
using namespace mrpt::slam;
using namespace mrpt::poses;
using namespace mrpt::math;
using namespace mrpt::gui;

using namespace std;

//this code is intended to be used to
//experiment with incorporating all the
//sensors into one program.


int main( int argc, char** argv )  {
	TSetOfMetricMapInitializers mapInit;
	mapInit.loadFromConfigFileName("map.ini", "MAP");
	mapInit.dumpToConsole();

	CMatrixDouble mat;
	//CPoint3D point(0,0,0);
	CMultiMetricMap map(&mapInit);

	map.m_gridMaps.front()->loadFromBitmapFile("test.bmp", .1, 0, 0);

	// Build voronoi:
	cout << "Building Voronoi diagram...\n";

	map.m_gridMaps.front()->buildVoronoiDiagram(0.5,0.3);


	// Show results:
	CImage img_grid;
	map.m_gridMaps.front()->getAsImage(img_grid);

	CImage img_voronoi;
	CMatrixDouble mat_voronoi;
	map.m_gridMaps.front()->getVoronoiDiagram().getAsMatrix(mat_voronoi);
	img_voronoi.setFromMatrix(mat_voronoi, false /* do normalization */ );

	// Show results:
	CDisplayWindow   win1("Grid map");
	win1.showImage(img_grid);

	CDisplayWindow   win2("Voronoi map");
	win2.showImage(img_voronoi);
	img_voronoi.saveToFile("voronoi.jpeg",100);

	mrpt::system::pause();
/*	CImage image(100,100,1,true);
	image.loadFromFile("test.bmp",1);
	CMatrixFloat imageMat;
	image.getAsMatrix(imageMat);

	map.m_gridMaps.front()->buildVoronoiDiagram(1, 1);
	map.m_gridMaps.front()->getVoronoiDiagram().getAsMatrix(mat);

	CImage voronoiImage(100,100,1,true);
	voronoiImage.setFromMatrix(mat,false);
	voronoiImage.saveToFile("voronoi.jpeg",100);
	cout << mat << endl;
*/
	return 0;
}
