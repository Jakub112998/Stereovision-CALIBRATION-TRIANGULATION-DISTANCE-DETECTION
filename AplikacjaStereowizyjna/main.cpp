#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/highgui/highgui_c.h>
#include <opencv2/aruco.hpp>
#include <opencv2/calib3d.hpp>
#include <cassert>
#include "csv.h"
#include <time.h>

#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <sstream>
#include <iostream>
#include <fstream>

using namespace std;
using namespace cv;

const float calibrationSquareDimension = 24; // meters 0.024f
const float arucoSquareDimension = 19;//meters 0.019f
const Size chessboardDimensions = Size(10, 6);

void createKnownBoardPosition(Size boardSize, float squareEdgeLength, vector<Point3f>& corners)
{
	for (int i = 0; i < boardSize.height; i++)
	{
		for (int j = 0; j < boardSize.width; j++)
		{
			corners.push_back(Point3f(j * squareEdgeLength, i * squareEdgeLength, 0.0f));
		}
	}
}

void getChessboardCorners(vector<Mat> images, vector<vector<Point2f>>& allFoundCorners, bool showResults = false)
{
	for (vector<Mat>::iterator iter = images.begin(); iter != images.end(); iter++)
	{
		vector<Point2f> pointBuf;
		bool found = findChessboardCorners(*iter, Size(9, 6), pointBuf, CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_NORMALIZE_IMAGE);
		if (found)
		{
			allFoundCorners.push_back(pointBuf);
		}
		if (showResults)
		{
			drawChessboardCorners(*iter, Size(9, 6), pointBuf, found);
			imshow("Looking for Corners", *iter);
			waitKey(0);
		}
	}
}

void createArucoMarkers()
{
	Mat outputMarker;

	Ptr<aruco::Dictionary> markerDictionary = aruco::getPredefinedDictionary(aruco::PREDEFINED_DICTIONARY_NAME::DICT_4X4_50);

	for (int i = 0; i < 50; i++)
	{
		aruco::drawMarker(markerDictionary, i, 500, outputMarker, 1);
		ostringstream convert;
		string imageName = "4X4Marker_";
		convert << imageName << i << ".jpg";
		imwrite(convert.str(), outputMarker);
	}
}

void cameraCalibration(vector<Mat> calibrationImages, Size boardSize, float squareEdgeLength, Mat& cameraMatrix, Mat& distanceCoreficience, vector<Mat>& rVectors, vector<Mat>& tVectors)
{
	vector<vector<Point2f>> checkerboardImageSpacePoints;
	getChessboardCorners(calibrationImages, checkerboardImageSpacePoints, false);

	vector<vector<Point3f>> worldSpaceCornerPoints(1);

	createKnownBoardPosition(boardSize, squareEdgeLength, worldSpaceCornerPoints[0]);
	worldSpaceCornerPoints.resize(checkerboardImageSpacePoints.size(), worldSpaceCornerPoints[0]);

	//vector<Mat> rVectors, tVectors;
	distanceCoreficience = Mat::zeros(8, 1, CV_64F); // zero matrix

	calibrateCamera(worldSpaceCornerPoints, checkerboardImageSpacePoints, boardSize, cameraMatrix, distanceCoreficience, rVectors, tVectors);
}

bool saveMatrix(string name, Mat cameraMatrix, Mat distanceCoefficients)
{
	ofstream outStream(name);
	if (outStream)
	{
		uint16_t rows = cameraMatrix.rows;
		uint16_t colums = cameraMatrix.cols;

		outStream << rows << endl;
		outStream << colums << endl;

		for (int r = 0; r < rows; r++)
		{
			for (int c = 0; c < colums; c++)
			{
				double value = cameraMatrix.at<double>(r, c);
				outStream << value << endl;
			}
		}

		rows = distanceCoefficients.rows;
		colums = distanceCoefficients.cols;

		outStream << rows << endl;
		outStream << colums << endl;

		for (int r = 0; r < rows; r++)
		{
			for (int c = 0; c < colums; c++)
			{
				double value = distanceCoefficients.at<double>(r, c);
				outStream << value << endl;
			}
		}
		outStream.close();
		return true;
	}
	return false;
}

bool loadMatrix(string name, Mat& camerMatrix, Mat& distanceCoefficients)
{
	ifstream inStream(name);
	if (inStream)
	{
		uint16_t rows;
		uint16_t colums;

		inStream >> rows;
		inStream >> colums;

		camerMatrix = Mat(Size(colums, rows), CV_64F);

		for (int r = 0; r < rows; r++)
		{
			for (int c = 0; c < colums; c++)
			{
				double read = 0.0f;
				inStream >> read;
				camerMatrix.at<double>(r, c) = read;
			}
		}
		//distanceCoefficients
		inStream >> rows;
		inStream >> colums;

		distanceCoefficients = Mat::zeros(rows, colums, CV_64F);

		for (int r = 0; r < rows; r++)
		{
			for (int c = 0; c < colums; c++)
			{
				double read; 0.0f;
				inStream >> read;
				distanceCoefficients.at<double>(r, c) = read;
			}
			inStream.close();
			return true;
		}
		return false;
	}
}

int stereoCalib(const char* imageList, int nx, int ny, int useUncalibrated)
{
	bool displayCorners = true;
	const int maxScale = 1;
	const float squareSize = calibrationSquareDimension;

	int N = nx * ny;
	cv::Size board_sz = cv::Size(nx, ny);
	vector<cv::Point3f> boardModel; // model markera
	vector<vector<cv::Point2f> > points[2];
	vector<cv::Point2f> corners[2];
	vector<cv::Point2f> allpoints[2];
	vector<vector<cv::Point3f> > objectPoints;
	bool found[2] = { false, false };
	cv::Size imageSize, imageSize1, imageSize2;
	cv::Mat R1, R2, P1, P2, R, T, E, F, Q, cameraMatrix[2], distCoeffs[2];
	bool showUndistorted = true;
	bool isVerticalStereo = false; // horiz or vert cams

	for (int i = 0; i < ny; i++)//wysokość
		for (int j = 0; j < nx; j++) {//szerokość
		boardModel.push_back(cv::Point3f((float)(i * squareSize), (float)(j * squareSize), 0.f)); // tworzę punkty 3D mojej szachownicy w przestrzeni
	}
	for(int d = 50; d<=5120; d=d+40)
	{
		found[0] = false; found[1] = false;

		cv::Mat img1 = cv::imread("2marker_left"+to_string(d)+".jpg");
		cv::Mat img2 = cv::imread("2marker_right" + to_string(d) + ".jpg");

		imageSize1 = img1.size();
		imageSize = imageSize1;
		imageSize2 = img2.size();
		cout << "wczytany obraz 1 ma rozmiar: " << imageSize1;
		cout << "\nwczytany obraz 2 ma rozmiar: " << imageSize2 << "\n";
		found[0] = cv::findChessboardCorners(img1, board_sz, corners[0]);
		found[1] = cv::findChessboardCorners(img2, board_sz, corners[1]);
		if (!found[0] || !found[1]) {
			cout << "\nKlatka nie została wczytana -----------------------------------------";
			continue;
		}
		if (found[0] && found[1]) {
			objectPoints.push_back(boardModel);//wzór
			points[0].push_back(corners[0]);//z lewej kamery
			points[1].push_back(corners[1]);//z prawej kamery
		}
	}
	cout << "\nsize objectPoints: " << objectPoints.size() << " ; size objectPoints[0]: " << objectPoints[0].size();
	cout << "\nsize points[0]: " << points[0].size();
	cout << "\nsize points[1]: " << points[1].size();

	cout << "\nRunning stereo calibration ...\n";
	cameraMatrix[0] = Mat(3, 3, CV_64F);
	cameraMatrix[1] = Mat(3, 3, CV_64F);
	distCoeffs[0] = Mat(1, 5, CV_64F);
	distCoeffs[1] = Mat(1, 5, CV_64F);
	R = Mat(3, 3, CV_64F);
	T = Mat(3, 1, CV_64F);
	E = Mat(3, 3, CV_64F);
	F = Mat(3, 3, CV_64F);
	stereoCalibrate(objectPoints, points[0], points[1], cameraMatrix[0], distCoeffs[0], cameraMatrix[1], distCoeffs[1],
		imageSize1, R, T, E, F, 0, cvTermCriteria(CV_TERMCRIT_ITER + CV_TERMCRIT_EPS, 100, 1e-5));

	cout << "cameraMatrix[0]: " << cameraMatrix[0] << " ; distCoeffs[0]: " << distCoeffs[0];
	cout << "cameraMatrix[1]: " << cameraMatrix[1] << " ; distCoeffs[1]: " << distCoeffs[1];

	cout << "\nRectification started" << endl;
		int nframes = (int)objectPoints.size();
	Rect validRoi[2];
	//if (!useUncalibrated) {
		//cout << "\n\nMETODA Bouqueta - 506\n\n";
	stereoRectify(cameraMatrix[0], distCoeffs[0], cameraMatrix[1], distCoeffs[1], imageSize1, R, T, R1, R2, P1, P2, Q,
		CALIB_ZERO_DISPARITY, 0, imageSize1, &validRoi[0], &validRoi[1]);
	//}
	/*else {
		cout << "\n\nMETODA HARLEY - 512\n\n";
		for (int i = 0; i < nframes; i++) {
			copy(points[0][i].begin(), points[0][i].end(), back_inserter(allpoints[0]));
			copy(points[1][i].begin(), points[1][i].end(), back_inserter(allpoints[1]));
		}
		F = findFundamentalMat(allpoints[0], allpoints[1], cv::FM_8POINT);
		cv::Mat H1, H2;
		cv::stereoRectifyUncalibrated(allpoints[0], allpoints[1], F, imageSize1, H1, H2, 3);
		R1 = cameraMatrix[0].inv() * H1 * cameraMatrix[0];
		R2 = cameraMatrix[1].inv() * H2 * cameraMatrix[1];
		P1 = cameraMatrix[0];
		P2 = cameraMatrix[1];
	}
	cout << "\nRectification completed" << endl;*/

	// TEST JAKOŚCI KALIBRACJI
	vector<cv::Point3f> lines[2];
	double avgErr = 0;
	for (int i = 0; i < nframes; i++) {
		vector<cv::Point2f>& pt0 = points[0][i];
		vector<cv::Point2f>& pt1 = points[1][i];
		cv::undistortPoints(pt0, pt0, cameraMatrix[0], distCoeffs[0], cv::Mat(), cameraMatrix[0]);
		cv::undistortPoints(pt1, pt1, cameraMatrix[1], distCoeffs[1], cv::Mat(), cameraMatrix[1]);
		cv::computeCorrespondEpilines(pt0, 1, F, lines[0]);
		cv::computeCorrespondEpilines(pt1, 2, F, lines[1]);
		double err, err_courrent=0;
		for (int j = 0; j < N; j++) {
			err = fabs(pt0[j].x * lines[1][j].x + pt0[j].y * lines[1][j].y + lines[1][j].z) +
				fabs(pt1[j].x * lines[0][j].x + pt1[j].y * lines[0][j].y + lines[0][j].z);
			err_courrent += err;
			avgErr += err;
		}
		err_courrent = err_courrent / N;
		cout << "\npara zdjec: " << i << " , err = " << err_courrent << "\n";
	}
	cout << "nframes: " << nframes << "\n";
	cout << "avg err = " << avgErr / (nframes * N) << endl;

	saveMatrix("KalibracjaKamery1", cameraMatrix[0], distCoeffs[0]);
	saveMatrix("KalibracjaKamery2", cameraMatrix[1], distCoeffs[1]);
	saveMatrix("R1R2", R1, R2);
	saveMatrix("P1P2", P1, P2);
	saveMatrix("RT", R, T);
}

cv::Mat computeProjMat(cv::Mat camMat, vector<cv::Mat> rotVec, vector<cv::Mat> transVec)
{
	cv::Mat rotMat(3, 3, CV_64F), RTMat(3, 4, CV_64F);
	//1. Convert rotation vector into rotation matrix 
	cv::Rodrigues(rotVec.at(0), rotMat);
	//2. Append translation vector to rotation matrix
	cv::hconcat(rotMat, transVec.at(0), RTMat);
	//3. Compute projection matrix by multiplying intrinsic parameter 
	//matrix (A) with 3 x 4 rotation and translation pose matrix (RT).
	return (camMat * RTMat);
}

int stereoVision(float arcuoSquareDimension, int nx, int ny, int useUncalibrated)
{
	bool found[2] = { false, false };
	cv::Mat R, T, E, F;
	cv::Mat R1(1, 4, CV_64F), R2(1, 4, CV_64F), P1(1, 4, CV_64F), P2(1, 4, CV_64F), Q(1, 4, CV_64F);
	//cv::Mat R1(1, 4, CV_64F), R2(1, 4, CV_64F), P1(3, 4, CV_64F), P2(3, 4, CV_64F), Q(1, 4, CV_64F);
	Mat cameraMatrix[2], distCoeffs[2], u1, u2;
	loadMatrix("KalibracjaKamery1", cameraMatrix[0], distCoeffs[0]);
	loadMatrix("KalibracjaKamery2", cameraMatrix[1], distCoeffs[1]);
	loadMatrix("R1R2", R1, R2);
	loadMatrix("P1P2", P1, P2);
	loadMatrix("RT", R, T);
	Mat rmap[2][2];
	vector<Vec3d> rvec1, tvec1;
	vector<Vec3d> rvec2, tvec2;

	//initUndistortRectifyMap(cameraMatrix[0], distCoeffs[0], R1, P1, { 640,480 }, CV_16SC2, rmap[0][0], rmap[0][1]);
	//initUndistortRectifyMap(cameraMatrix[1], distCoeffs[1], R2, P2, { 640,480 }, CV_16SC2, rmap[1][0], rmap[1][1]);
	cv::Size board_sz = cv::Size(nx, ny);
	cv::Ptr<cv::aruco::Dictionary> markerDictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_250);
	Mat frame1, frame2;
	Mat img1;
	Mat img2;
	cv::Mat imgboth;
	aruco::DetectorParameters parameters;
	vector<cv::Point2f> markerPoints[2];

	int count = 0;
	bool fist_measure_3 = true, fist_measure_0 = true;
	double PP_X_3=0, PP_X_0=0, PP_Y_3=0, PP_Y_0=0, PP_Z_3=0, PP_Z_0=0, PP_D_3=0, PP_D_0=0;
	vector<pomiar> lista;//P
	int numer_zdjęcia = 2;
	//cv::stereoRectify(cameraMatrix[0], distCoeffs[0], cameraMatrix[1], distCoeffs[1], u1.size(), R, T, R1, R2, P1, P2, Q, CALIB_ZERO_DISPARITY);
	int n = 5;//pomiar nr
	int i = 0;//P //+1
	for (int f = 0; f <= 611; f++)//pomiar4 - 596
	{
		vector<vector<cv::Point2f>> markerCorners[2];
		vector<int> markerIds1, markerIds2;
		found[0] = false; found[1] = false;

		//if (i == 333) Stosowane w przypadku wadliwych zdjęć
		//{
			//saveToCSV(lista, n);
		//}
		pomiar p;
		p.chwila = i;

		img1 = cv::imread("marker_right_n3.jpg");// Do wstępnej weryfikacji działania programu
		img2 = cv::imread("marker_left_n3.jpg");// Do wstępnej weryfikacji działania programu
		//img1 = cv::imread(to_string(numer_zdjęcia)+"marker_left" +std::to_string(i)+".jpg");//P
		//img2 = cv::imread(to_string(numer_zdjęcia)+ "marker_right" +std::to_string(i)+".jpg");//P
		if (img1.empty() || img2.empty())
			break;//
		cout << "\nWczytano: " << to_string(numer_zdjęcia)+"marker_left" + std::to_string(i) + ".jpg\n";
		cout << "Wczytano: " << to_string(numer_zdjęcia)+"marker_right" + std::to_string(i) + ".jpg";

		aruco::detectMarkers(img1, markerDictionary, markerCorners[0], markerIds1);
		aruco::detectMarkers(img2, markerDictionary, markerCorners[1], markerIds2);

		i++;
		if (markerCorners[0].size() > 0) {//jeśli wykryło
			found[0] = true;
		}
		if (markerCorners[1].size() > 0) {//jeśli wykryło
			found[1] = true;
		}
		if (!found[0] || !found[1]) {
			p.markerIds = -1;
			p.X = 0; p.Y = 0; p.Z = 0; p.distance = 0;
			lista.push_back(p);
			continue;
		}
		if (markerIds1.size() > 0 || markerIds2.size() > 0)
		{
			cv::aruco::estimatePoseSingleMarkers(markerCorners[0], arcuoSquareDimension, cameraMatrix[0], distCoeffs[0], rvec1, tvec1);
			cv::aruco::estimatePoseSingleMarkers(markerCorners[1], arcuoSquareDimension, cameraMatrix[1], distCoeffs[1], rvec2, tvec2);
			if (markerIds1.size() == markerIds2.size())
			{
				double* center1_x = new double[markerCorners[0].size()];
				double* center1_y = new double[markerCorners[0].size()];
				double* center2_x = new double[markerCorners[0].size()];
				double* center2_y = new double[markerCorners[0].size()];
				double* distance = new double[markerCorners[0].size()];
				double* wResult = new double[markerCorners[0].size()];
				double* realX = new double[markerCorners[0].size()];
				double* realY = new double[markerCorners[0].size()];
				double* realZ = new double[markerCorners[0].size()];
				cv::Mat* imgPoints1 = new cv::Mat[markerCorners[0].size()];
				cv::Mat* imgPoints2 = new cv::Mat[markerCorners[0].size()];
				cv::Mat* out = new cv::Mat[markerCorners[0].size()];
				double* x1 = new double[markerCorners[0].size()];
				double* y1 = new double[markerCorners[0].size()];
				double* x2 = new double[markerCorners[0].size()];
				double* y2 = new double[markerCorners[0].size()];
				double* do_druku = new double[markerCorners[0].size()];

				for (int z = 0; z < markerCorners[0].size(); z++)
				{
					cv::undistort(img1, u1, cameraMatrix[0], distCoeffs[0]);
					cv::undistort(img2, u2, cameraMatrix[1], distCoeffs[1]);
					cout << "\nP1: " << P1 << " \nP2: " << P2;
					cv::stereoRectify(cameraMatrix[0], distCoeffs[0], cameraMatrix[1], distCoeffs[1], u1.size(), R, T, R1, R2, P1, P2, Q, CALIB_ZERO_DISPARITY);
					aruco::detectMarkers(u1, markerDictionary, markerCorners[0], markerIds1);
					aruco::detectMarkers(u2, markerDictionary, markerCorners[1], markerIds2);

					cout << "\nR: " << R << "\nT: " << T;

					center1_x[z] = (markerCorners[0][z][0].x + markerCorners[0][z][1].x + markerCorners[0][z][2].x + markerCorners[0][z][3].x) / 4;
					center1_y[z] = (markerCorners[0][z][0].y + markerCorners[0][z][1].y + markerCorners[0][z][2].y + markerCorners[0][z][3].y) / 4;
					cout << "\ncenter1: " << center1_x[z] << "; " << center1_y[z];
					center2_x[z] = (markerCorners[1][z][0].x + markerCorners[1][z][1].x + markerCorners[1][z][2].x + markerCorners[1][z][3].x) / 4;
					center2_y[z] = (markerCorners[1][z][0].y + markerCorners[1][z][1].y + markerCorners[1][z][2].y + markerCorners[1][z][3].y) / 4;
					cout << "\ncenter2: " << center2_x[z] << "; " << center2_y[z];
					out[z] = cv::Mat(3, 1, CV_64F);//TU ZMIANA
					imgPoints1[z] = cv::Mat(2, 1, CV_64F);
					imgPoints1[z] = { center1_x[z], center1_y[z] };
					imgPoints2[z] = cv::Mat(2, 1, CV_64F);
					imgPoints2[z] = { center2_x[z], center2_y[z] };
					//cout << "\nimgPoints1[z]: " << imgPoints1[z] << " \nimgPoints2[z]: " << imgPoints2[z];
					cout << "\nP1: " << P1 << " \nP2: " << P2;
					//cout << "\ncameraMatrix[0]: " << cameraMatrix[0] << " \ncameraMatrix[1]: " << cameraMatrix[1];

					triangulatePoints(P1, P2, imgPoints2[z], imgPoints1[z], out[z]);
					
					wResult[z] = out[z].at<double>(3, 0);
					realX[z] = out[z].at<double>(0, 0) / wResult[z];
					realY[z] = (out[z].at<double>(1, 0) / wResult[z]);
					realZ[z] = (out[z].at<double>(2, 0) / wResult[z]);

					int Ids = markerIds2.size();
					if (markerIds1.size() > markerIds2.size())
						Ids = markerIds1.size();
					for (int i = 0; i < Ids; i++) {
						if (i < markerIds1.size())
							cv::aruco::drawDetectedMarkers(u1, markerCorners[0], markerIds1);
						if (i < markerIds2.size())
							cv::aruco::drawDetectedMarkers(u2, markerCorners[1], markerIds2);
					}

					distance[z] = sqrt((realX[z] * realX[z]) + (realY[z] * realY[z]) + (realZ[z] * realZ[z]));

					cout << "\nwResult[z]: " << wResult[z] << " realX[z]: " << realX[z] << "; realY[z]: " << realY[z] << "; realZ[z]: " << realZ[z];
					cout << "\nDystans od: " << markerIds1[z] << " dystans: " << distance[z];// To działa
					//pomiary:
					p.X = (realX[z]); p.Y = (realY[z]); p.Z = (realZ[z]); p.distance = (distance[z]); p.markerIds = markerIds1[z];
					lista.push_back(p);
				}
			}
			else
			{
				p.markerIds = -1;
				p.X = 0; p.Y = 0; p.Z = 0; p.distance = 0;
				lista.push_back(p);
				cout << "ArUco markers - not detected.\n";
				continue;//
			}
			//pokazuję obrazy z dwóch kamer jednocześnie-----------------------
			cv::Mat imgboth(u1.rows, u1.cols * 2, u1.type());
			u1.copyTo(cv::Mat(imgboth, cv::Rect(0, 0, u1.cols, u1.rows)));
			u2.copyTo(cv::Mat(imgboth, cv::Rect(u1.cols, 0, u2.cols, u2.rows)));
			for (int j = 0; j < imgboth.rows; j += 16) {
				Point p1 = Point(0, j);
				Point p2 = Point(imgboth.cols * 2, j);
				line(imgboth, p1, p2, CV_RGB(255, 0, 0));
			}

			//CZĘŚĆ OBRAZOWA
			/*std::string name = "marker_left_z" + std::to_string(count)// + "_dystans_" + std::to_string(do_druku)
				+ ".jpg";
			cv::imwrite(name, u1);
			name = "marker_right_z" + std::to_string(count)// +"_dystans_" + std::to_string(do_druku)
				+ ".jpg";
			cv::imwrite(name, u2);*/
			//cv::namedWindow("rectified", cv::WINDOW_NORMAL);
			//cv::imshow("rectified", imgboth);

			//imshow("aruco markers example 1", u1);
			//imshow("aruco markers example 2", u2);
			//imshow("img1", img1);
			//imshow("img2", img2);
			//if (waitKey(10) == 27) break;
			//--testy
			/*cout << " ; ZAPISUJE! scena numer: " << count << "\n";
			Mat temp1, temp2;
			std::string name = "marker_left_z" + std::to_string(count)// + "_dystans_" + std::to_string(do_druku)
			+ ".jpg";
			std::string name_n = "marker_left_n" + std::to_string(count)// + "_dystans_" + std::to_string(do_druku)
				+ ".jpg";
			const char* c = name.c_str();
			cv::imwrite(name, u1);
			//cv::imwrite(name_n, img1);
			name = "marker_right_z" + std::to_string(count)// +"_dystans_" + std::to_string(do_druku)
			+ ".jpg";
			name_n = "marker_right_n" + std::to_string(count)// + "_dystans_" + std::to_string(do_druku)
				+ ".jpg";
			cv::imwrite(name, u2);*/
			//cv::imwrite(name_n, img2);
			//--testy
			count++;
		}
	}
	saveToCSV(lista, n);
	return 1;
}

int adjusting()
{
	Mat frame1, frame2;
	VideoCapture vid1, vid2;
	if (!vid1.open(0) || !vid2.open(2))
		cout << "kamera 1 lub 2 nie uruchomiła się";
	vid1 >> frame1;
	vid2 >> frame2;
	Mat img1, img2;
	for (;;)
	{
		img1 = frame1;
		img2 = frame2;
		if (!vid1.read(img1) || !vid2.read(img2))
			continue;
		int szerokosc = frame1.size().width;
		int wysokosc = frame1.size().height;
		cv::Point poczatek1, poczatek2;
		cv::Point koniec1, koniec2;
		poczatek1 = { 0, wysokosc / 2 };
		koniec1 = { szerokosc, wysokosc / 2 };
		poczatek2 = { szerokosc/2, 0};
		koniec2 = { szerokosc / 2, wysokosc};

		szerokosc = frame2.size().width;
		wysokosc = frame2.size().height;
		cv::Point poczatek3, poczatek4;
		cv::Point koniec3, koniec4;
		poczatek3 = { 0, wysokosc / 2 };
		koniec3 = { szerokosc, wysokosc / 2 };
		poczatek4 = { szerokosc / 2, 0 };
		koniec4 = { szerokosc / 2, wysokosc };
		cv::line(img1, poczatek1, koniec1, (255, 255, 255), 1);
		cv::line(img1, poczatek2, koniec2, (255, 255, 255), 1);
		cv::line(img2, poczatek3, koniec3, (255, 255, 255), 1);
		cv::line(img2, poczatek4, koniec4, (255, 255, 255), 1);
		imshow("Webcam1", img1);
		imshow("Webcam2", img2);
		if (waitKey(10) == 27) break;
	}
	return 1;
}

int main(int argv, char** argc)
{
	//adjusting();
	//createArucoMarkers();
	record(2);
	int useUncalibrated = 2;//0 - Bouquetta lub 2 - Harley
	//cameraCalibrationProcess();
	//stereoCalib("list.txt", 10, 6, useUncalibrated);
	stereoVision(arucoSquareDimension, 4, 4, useUncalibrated);
	return 0;
}