class PID_position
{
private:
    double kp;//比例系数
    double ki;//积分系数
    double kd;//微分系数
    
    double e;//误差
    double e_pre;//上一次误差
    double integral;//积分项
public:
    PID_position();
    ~PID_position(){};
    PID_position(double p,double i,double d);
    double pid_control(double e);//执行PID控制
    void pid_show();//显示PID控制器的内部参数
};