#include "PID_position.h"
#include <iostream>
using namespace std;

//位置PID
PID_position::PID_position():kp(1),ki(0),kd(0),integral(0)
{
    e=0;
    e_pre=e;
}
PID_position::PID_position(double p,double i,double d):kp(p),ki(i),kd(d),integral(0)
{
   e=0;
   e_pre=e;
}
double PID_position::pid_control(double e)
{
    double u;
    integral+=e;
    u=kp*e+ki*integral+kd*(e-e_pre);
    e_pre=e;
    return u;
}
void PID_position::pid_show()
{
    cout<<"The infomation of this position PID controller is as following:"<<endl;
    cout<<"       Kp="<<kp<<endl;
    cout<<"       Ki="<<ki<<endl;
    cout<<"       Kd="<<kd<<endl;

    cout<<" integral="<<integral<<endl;
    cout<<"        e="<<e<<endl;
    cout<<"    e_pre="<<e_pre<<endl;
}