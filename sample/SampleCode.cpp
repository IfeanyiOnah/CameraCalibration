#include "CameraCalibration.h"

using namespace std;
using namespace cv;

int main() {
	// indtantiate the object of the demosaic class
	Calib::CameraCalibration *objCalib = new Calib::CameraCalibration();

	//execute the algorithm
	objCalib->Calibrate("calConfig.xml");

	//delete the point to the demosaic class
	delete objCalib;

	return 0;
}