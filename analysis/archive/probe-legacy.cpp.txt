// Reproduktionshilfe für die Codeanalyse. Ändert die Firmware nicht.
#include <Arduino.h>
#include "SerialCommands.h"
#include <stdio.h>
unsigned long testMicros=0,testRisingEdges[256]={};int testPins[256]={};FakeSerial Serial;
using namespace scara;
void tick(MotorController& m){testMicros+=config::TICK_US;m.tick();}
void finish(MotorController& m,Movement& movement){for(unsigned long n=0;n<3000000&&movement.busy();++n){movement.update();tick(m);}}
void init(MotorController& m){m.begin();Movement move(m);Joints q;q.q[1]=config::PI_VALUE/2;q.q[2]=-config::PI_VALUE/2;q.q[3]=30;move.moveJoints(q,5);finish(m,move);}
int main(){
 {MotorController m;m.begin();Movement move(m);Pose p;p.x=210;p.y=145;p.z=30;
 printf("HOME-logical-zero MOVEL: %s\n",errorName(move.moveLine(p,1,false,false,true)));}
 {MotorController m;init(m);Movement move(m);Pose p;p.x=210;p.y=145;p.z=30;
 Joints target;Kinematics k;k.inverse(p,1,m.position(),target);
 move.moveJoints(target,0.1);finish(m,move);printf("same endpoints MOVEJ: T_fahrt=%.3f, error=%s\n",move.duration(),errorName(move.error()));}
 {MotorController m;init(m);Movement move(m);Pose p;p.x=210;p.y=145;p.z=30;
 move.moveLine(p,0.1,false,false,true);finish(m,move);printf("same endpoints MOVEL: T_fahrt=%.3f, error=%s\n",move.duration(),errorName(move.error()));}
 {MotorController m;init(m);Movement move(m);Pose p;p.x=210;p.y=145;p.z=30;
 move.moveLine(p,0.000000001,false,false,true);for(unsigned n=0;n<1000&&move.state()==Movement::Planning;++n)move.update();
 printf("tiny positive duration: state=%u, T_fahrt=%.3f, error=%s\n",unsigned(move.state()),move.duration(),errorName(move.error()));}
 {MotorController m;init(m);Kinematics k;Joints start=m.position();Pose a=k.forward(start),b;b.x=210;b.y=145;b.z=30;
 const int N=20000;static double q[N+1][4];QuinticProfile p;p.restToRest(1);Joints seed=start;
 for(int n=0;n<=N;++n){Joints out;Error e=k.inverse(interpolate(a,b,p.evaluate(double(n)/N).s),1,seed,out);if(e!=Error::None)return 2;for(int i=0;i<4;++i)q[n][i]=out.q[i];seed=out;}
 double T=0;for(int i=0;i<4;++i){double v=0,acc=0;for(int n=1;n<N;++n){v=fmax(v,fabs((q[n+1][i]-q[n-1][i])*N/2));acc=fmax(acc,fabs((q[n+1][i]-2*q[n][i]+q[n-1][i])*N*N));}double t=fmax(v/config::AXES[i].MaxSpeed,sqrt(acc/config::AXES[i].MaxAcceleration));T=fmax(T,t);}
 printf("dense numerical MOVEL estimate (not a continuous bound): %.3f s\n",T);}
}
