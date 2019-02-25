#pragma once
#ifndef CameraCalibration_H_
#define CameraCalibration_H_

#include "Settings.h"

namespace Calib {

#define ExportCalibration _declspec(dllexport)


	class ExportCalibration CameraCalibration
	{
	public:
		CameraCalibration() {}
		~CameraCalibration() {}

	private:

		enum { DETECTION = 0, CAPTURING = 1, CALIBRATED = 2 };


		static void help();

		//! [compute_errors]
		static double computeReprojectionErrors(const vector<vector<cv::Point3f> >& objectPoints,
			const vector<vector<cv::Point2f> >& imagePoints,
			const vector<cv::Mat>& rvecs, const vector<cv::Mat>& tvecs,
			const cv::Mat& cameraMatrix, const cv::Mat& distCoeffs,
			vector<float>& perViewErrors, bool fisheye);

		//! [compute_errors]
		//! [board_corners]
		static void calcBoardCornerPositions(cv::Size boardSize, float squareSize, vector<cv::Point3f>& corners,
			Settings::Pattern patternType /*= Settings::CHESSBOARD*/);

		//! [board_corners]
		static bool runCalibration(Settings& s, cv::Size& imageSize, cv::Mat& cameraMatrix, cv::Mat& distCoeffs,
			vector<vector<cv::Point2f> > imagePoints, vector<cv::Mat>& rvecs, vector<cv::Mat>& tvecs,
			vector<float>& reprojErrs, double& totalAvgErr);

		// Print camera parameters to the output file
		static void saveCameraParams(Settings& s, cv::Size& imageSize, cv::Mat& cameraMatrix, cv::Mat& distCoeffs,
			const vector<cv::Mat>& rvecs, const vector<cv::Mat>& tvecs,
			const vector<float>& reprojErrs, const vector<vector<cv::Point2f> >& imagePoints,
			double totalAvgErr, int angle);

		static bool runCalibrationAndSave(Settings& s, cv::Size imageSize, cv::Mat& cameraMatrix, cv::Mat& distCoeffs,
			vector<vector<cv::Point2f> > imagePoints, int angle);

		static cv::Mat imgUndist(cv::Mat view, cv::Mat cameraMatrix, cv::Mat distCoef);
		static void imgSplit(cv::Mat input, cv::Mat &im0, cv::Mat &im45, cv::Mat &im90, cv::Mat &im135, std::string method="sep");

	public:

		int Calibrate(const string &inputSettingsFile);
	};

}

#endif
