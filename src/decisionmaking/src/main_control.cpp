#include <ros/ros.h>
#include <iostream>
#include <decisionmaking/decisionmaking.h>

using namespace std;
using namespace ros;

int main(int argc, char**argv)
{
    init(argc, argv, "main_control");
    DecisionMaking decisionmaking;
    decisionmaking.Normal_num = 1;
    decisionmaking.place_num  = 1;
    Rate r(30);
    while(ok())
    {
        spinOnce();
        r.sleep();
    }
    
    return 0;
}