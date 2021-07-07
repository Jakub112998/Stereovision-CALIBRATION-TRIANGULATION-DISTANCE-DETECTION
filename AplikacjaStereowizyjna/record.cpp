#include "csv.h"
#include <cassert>
#include <iostream>
#include <string.h>

using namespace std;

int record(int numer_prostej)
{
	Mat frame1, frame2;
	VideoCapture vid1, vid2;
	if (!vid1.open(0) || !vid2.open(2)) // sprawdzam czy mam w³¹czone 2 kamery jednoczeœnie
		cout << "wiersz 593 - kamera 1 lub 2 nie uruchomi³a siê";
	vid1 >> frame1;
	vid2 >> frame2;

	int count = 0;
	for (;;) {
		vid1 >> frame1;
		vid2 >> frame2;
		imshow("Frame_left", frame1);
		imshow("Frame_right", frame2);

		std::string name = std::to_string(numer_prostej) + "marker_left" + std::to_string(count)// + "_dystans_" + std::to_string(do_druku)
			+ ".jpg";
		const char* c = name.c_str();
		cv::imwrite(name, frame1);
		name = std::to_string(numer_prostej) + "marker_right" + std::to_string(count)// +"_dystans_" + std::to_string(do_druku)
			+ ".jpg";
		cv::imwrite(name, frame2);
		count++;
	}
	return 0;
}