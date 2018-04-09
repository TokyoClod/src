#include "PID_position.h"
#include <iostream>
using namespace std;

//位置PID
PID_position::PID_position():kp(0),ki(0),kd(0),target(0),actual(0),integral(0)
{
    e=target-actual;
    e_pre=e;
}
PID_position::PID_position(double p,double i,double d):kp(p),ki(i),kd(d),target(0),actual(0),integral(0)
{
   e=target-actual;
   e_pre=e;
}
double PID_position::pid_control(double tar,double act)
{
    double u;
    target=tar;
    actual=act;
    e=target-actual;
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
    cout<<"   target="<<target<<endl;
    cout<<"   actual="<<actual<<endl;
    cout<<"        e="<<e<<endl;
    cout<<"    e_pre="<<e_pre<<endl;
}