/*!
  \example get_distance.c �����f�[�^���擾����

  \author Satofumi KAMIMURA

  $Id: get_distance.cpp 1933 2010-10-05 14:09:41Z satofumi $
*/

#include "Urg_driver.h"
#include "Connection_information.h"
#include "math_utilities.h"
#include <iostream>
#include <fstream>
#include <math.h>

using namespace qrk;
using namespace std;


namespace
{
	struct XYZpoint {
		float x;
		float y;
		//int z;

	};

	XYZpoint CalculatePoint(const Urg_driver& urg, int scanNo, int distance, int y_angle)
	{
		XYZpoint newPoint;
		//float rollAngle = 0.25 * (scanNo - 45.125) ;
		double deg = urg.index2rad(scanNo);
		newPoint.x = cos(deg) * distance;
		newPoint.y = sin(deg) * distance;
		return newPoint;		
	} 
    void print_data(const Urg_driver& urg,
                    const vector<long>& data, long time_stamp)
    {
#if 1
        // �O���̃f�[�^�݂̂�\��
        int front_index = urg.step2index(0);
		ofstream outFile;
		outFile.open("output.txt");
		for(int i = 0; i < data.size(); i++)
		{
        cout << data[i] << " [mm], ("
             << time_stamp << " [msec])" << endl;

		outFile << i << ": " << CalculatePoint(urg, i, data[i], 0).x << " X " << CalculatePoint(urg, i, data[i], 0).y
			<< "Y " << endl;
		
		}
		outFile.close();
#else
        // �S�Ẵf�[�^�� X-Y �̈ʒu��\��
        long min_distance = urg.min_distance();
        long max_distance = urg.max_distance();
        size_t data_n = data.size();
        for (size_t i = 0; i < data_n; ++i) {
            long l = data[i];
            if ((l <= min_distance) || (l >= max_distance)) {
                continue;
            }

            double radian = urg.index2rad(i);
            long x = static_cast<long>(l * cos(radian));
            long y = static_cast<long>(l * sin(radian));
            cout << "(" << x << ", " << y << ")" << endl;
        }
        cout << endl;
#endif
    }
}


int main(int argc, char *argv[])
{
    Connection_information information(argc, argv);

    // �ڑ�
    Urg_driver urg;
    if (!urg.open(information.device_or_ip_name(),
                  information.baudrate_or_port_number(),
                  information.connection_type())) {
        cout << "Urg_driver::open(): "
             << information.device_or_ip_name() << ": " << urg.what() << endl;
        return 1;
    }

    // �f�[�^�擾
#if 1
    // �f�[�^�̎擾�͈͂�ύX����ꍇ
    urg.set_scanning_parameter(urg.deg2step(-135), urg.deg2step(+135), 0);
#endif
    enum { Capture_times = 1 };
    urg.start_measurement(Urg_driver::Distance, Capture_times, 0);
    for (int i = 0; i < Capture_times; ++i) {
        vector<long> data;
        long time_stamp = 0;

        if (!urg.get_distance(data, &time_stamp)) {
            cout << "Urg_driver::get_distance(): " << urg.what() << endl;
            return 1;
        }
        print_data(urg, data, time_stamp);
		
    }

#if defined(URG_MSC)
    getchar();
#endif
    return 0;
}
