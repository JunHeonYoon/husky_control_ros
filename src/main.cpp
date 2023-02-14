#include "husky_controller.h"
#include <stdio.h>

using namespace std; 
using namespace Eigen;

void printState(VectorXd& pose, VectorXd& vel)
{
    cout<<"Position:\n\t\t"<<"x : "<<pose(0)<<"\n\t\ty : "<<pose(1)<<"\n\t\tth: "<<pose(2)*180/M_PI<<endl;
    cout<<"Velocity:\n\t\t"<<"x : "<<vel(0)<<"\n\t\ty : "<<vel(1)<<"\n\t\tth: "<<vel(2)*180/M_PI<<endl;
    cout<<"\n\n"<<endl;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_node_vrep");
    ros::NodeHandle nh("~");

    double ctrl_hz = 50;
    controller_interface obj(nh, ctrl_hz);

    VectorXd desired_velocity(2);
    desired_velocity(0) = 0.25;  // linear velocity
    desired_velocity(1) = 0.125; // angular velocity

    VectorXd pose(3), velocity(3);

    sleep(1); obj.vrepStart();
    sleep(1); obj.vrepEnableSyncMode();
    sleep(1);

    while(ros::ok())
    {
        obj.read_vrep();
        obj.write_vrep(desired_velocity);
        obj.wait();
        obj.Getstate(pose, velocity);
        printState(pose, velocity);
    }
    obj.vrepStop();
    sleep(1);
    return 0;
}
