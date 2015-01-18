/*!
  \example get_distance.c �����f�[�^���擾����

  \author Satofumi KAMIMURA

  $Id: get_distance.cpp,v 2a7545b3f397 2014/08/27 01:26:02 jun $
*/

#include "Urg_driver.h"
#include "Connection_information.h"
#include "math_utilities.h"
#include <iostream>

using namespace qrk;
using namespace std;


namespace
{
    void print_data(const Urg_driver& urg,
                    const vector<long>& data, long time_stamp)
    {
#if 1
        // �O���̃f�[�^�݂̂�\��
        int front_index = urg.step2index(0);
        cout << data[front_index] << " [mm], ("
             << time_stamp << " [msec])" << endl;

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
    urg.set_scanning_parameter(urg.deg2step(-90), urg.deg2step(+90), 0);
#endif
    enum { Capture_times = 10 };
    urg.start_measurement(Urg_driver::Distance, Urg_driver::Infinity_times, 0);
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
