#ifndef _CHECKPMVSPOINT_HPP_
#define _CHECKPMVSPOINT_HPP_

#include <iostream>
#include <string>
#include <fstream>
#include <vector>
#include <map>
#include <iterator>
#include <Eigen/Dense>
#include <cctype>	  //isdigit()	
#include <cstdlib>    //atoi()
#include <limits>

#include <atlimage.h>

using namespace Eigen;

struct point_info
{
	size_t pointID;
	size_t pointvisibleimages;
	size_t imageID;
	double wx;     //point's world x coord;
	double wy;     //point's world y coord;
	double wz;	   //point's world x coord;
	double px;     //point's pixel x coord;
	double py;     //point's pixel y coord;
};

typedef int imageID;
typedef int pointID;
typedef std::vector<MatrixXd> cameraMatrix;
typedef std::vector<point_info> Point;

class cmpvsp
{
public:
	cmpvsp(std::string pmvspath)
	{
		init(pmvspath);
		ReadCameraParameter();
		ReadOptionPatch();
	}


	void GetCameraParameter(size_t CameraID);
	void GetPointWorldCoord(size_t PointID);
	Point GetPointImageCoord(size_t PointID);
	void CutOutPointVisibleImage(size_t PointID);

protected:
private:

	std::string pmvspath_;
	std::string pmvsmodels_;
	std::string pmvstxtpath_;
	std::string pmvsvisualize_;
	cameraMatrix cameraMatrix_;
	Point point_;

	void init(std::string pmvspath);
	void ReadCameraParameter();
	void ReadOptionPatch();

};

#endif