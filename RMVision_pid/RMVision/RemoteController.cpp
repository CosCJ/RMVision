/*******************************************************************************************************************
Copyright 2017 Dajiang Innovations Technology Co., Ltd (DJI)

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated
documentation files(the "Software"), to deal in the Software without restriction, including without limitation
the rights to use, copy, modify, merge, publish, distribute, sublicense, and / or sell copies of the Software, and
to permit persons to whom the Software is furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all copies or substantial portions of
the Software.
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO
THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF
CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
IN THE SOFTWARE.
*******************************************************************************************************************/

#include "RemoteController.hpp"
#include <unistd.h>
#include <termios.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include "serial.h"

using namespace std;


float angle_yaw;
float angle_pitch;


void RemoteController::paraReceiver(){
    while(1){
        praseDatafromCar();
        usleep(1000);
    }
}


void RemoteController::praseDatafromCar(){
    uchar buf[150]={0};
    size_t bytes = 0;
    ioctl(fd_car, FIONREAD, &bytes);
    if(bytes > 0 && bytes < 150)
        bytes = read(fd_car, buf, bytes);
    else if(bytes >= 150)
        bytes = read(fd_car, buf, 150);
    else
        return;

    praseData(buf, bytes);
}


std::string RemoteController::chToHex(const uchar ch)
{
    const std::string hex = "0123456789ABCDEF";

    std::stringstream ss;
    ss << hex[ch >> 4] << hex[ch & 0xf];

    return ss.str();
}



void RemoteController::praseData(const uchar * data, int size){
    if(size < 6)
        return;
    int start = 0;
    int angle_data=0;

    while(1){
        if (start >= size - 2)
            return;

        std::string SOF = chToHex(data[start]);
        std::string END = chToHex(data[start+5]);

        if(SOF == "F5" && END == "F6"){
            break;
        }
        ++start;
    }
    start++;

    cout<<"发送数据:    "<<endl;
    short yaw = (short)((uchar)data[start + 1] | (unsigned short)data[start]<<8);
    short pitch = (short)((uchar)data[start + 3] | (unsigned short)data[start + 2]<<8);
    cout<<yaw/100<<endl;
    cout<<pitch/100 <<endl;

     angle_yaw = yaw/100;
     angle_pitch = pitch/100;


    ++start;  // idx of FE
    if (start + 1 < size){
        praseData(data + start + 1, size - start - 1);
    }
}


