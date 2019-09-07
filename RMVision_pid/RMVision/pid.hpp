#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H
//位置式PID
class PID_position
{
public:
    float kp;//比例系数
    float ki;//积分系数
    float kd;//微分系数
    float target;//目标值
    float actual;//实际值
    float e;//误差
    float e_pre;//上一次误差

public:
    float integral;//积分项
public:
    PID_position();
  //  ~PID_position();
    PID_position(float p,float i,float d);
    float pid_control(float );//执行PID控制
    void pid_show();//显示PID控制器的内部参数
    void clear();
};




//位置PID
PID_position::PID_position():kp(0),ki(0),kd(0),target(0),actual(0),integral(0)
{
    e=target-actual;
    e_pre=e;
}
PID_position::PID_position(float p,float i,float d):kp(p),ki(i),kd(d),target(0),actual(0),integral(0)
{

   e=target-actual;
   e_pre=e;
}

void PID_position::clear()
{
    integral = 0;
    e = 0;
    e_pre = 0;
}

float PID_position::pid_control(float e1)
{
    float u;
    e = e1;
    integral+=e;
    u=kp*e+ki*integral+kd*(e-e_pre);
    e_pre=e;
    return u;
}
void PID_position::pid_show()
{
    using std::cout;
    using std::endl;
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



#endif // PID_CONTROLLER_H

