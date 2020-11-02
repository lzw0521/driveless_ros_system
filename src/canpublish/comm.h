#ifndef COMM_H
#define COMM_H
#include <sstream>

#define GETFRAMEID 0x18FF032A
#define GETSTATUSFRAMEID 0x18FF042A
#define SETFRAMEID 0x0CFF062B
#define CAN_DATA_LEN 8

#define BAUD_1000 0x1400
#define BAUD_800  0x1600
#define BAUD_666  0xB680
#define BAUD_500  0x1C00
#define BAUD_400  0xFA80
#define BAUD_250  0x1C01
#define BAUD_200  0xFA81
#define BAUD_125  0x1C03
#define BAUD_100  0x1C04

#define OFFSET_VEL 1000
#define OFFSET_ANGLE 900

template<class T>
void convertValue(T&value,const std::string &s)
{
    std::stringstream ss(s);
    ss>>value;

}

void deleteAllMark(std::string &s,const std::string strMask)
{
    size_t nSize = strMask.size();
    while(1)
    {
        size_t pos = s.find(strMask);
        if(pos == std::string::npos)
        {
            break;
        }

        s.erase(pos, nSize);
    }
}




#endif // COMM_H
