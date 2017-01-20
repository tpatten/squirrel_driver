#include <ros/ros.h>
#include <iostream>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Int32.h>
#include <boost/thread.hpp>

#define MOVECOMMANDTOPIC "/real/robotino/joint_control/goto"
#define PTPCOMMANDTOPIC "/real/robotino/joint_control/ptp"
#define MODETOPIC "/real/robotino/settings/switch_mode"
#define DONTCARE nan("1")

using namespace std;
void doWork();


ros::Publisher pubMode;
ros::Publisher pubMove;
ros::Publisher pubPtp;
std_msgs::Float64MultiArray command;
std_msgs::Int32 mode;


bool sendCommands=false;
int main(int argc, char** args) {


    ros::init(argc, args, "uibk_arm_demo"); sleep(1);
    ros::NodeHandle node;

    pubMove = node.advertise<std_msgs::Float64MultiArray>(MOVECOMMANDTOPIC, 1);
    pubMode = node.advertise<std_msgs::Int32>(MODETOPIC, 1);
    //pubPtp = node.advertise<std_msgs::Float64MultiArray>(PTPCOMMANDTOPIC, 1);
    boost::thread* workThread;
    workThread = new boost::thread(boost::bind(doWork));


    mode.data = 10;
    std::cout << "press any key to set the  mode" << std::endl;
    getchar();
    pubMode.publish(mode);

    string str;
    int cnt=0;
    command.data = {DONTCARE,DONTCARE, DONTCARE, DONTCARE, DONTCARE, DONTCARE, DONTCARE, DONTCARE};
    char type;
//    while (!(type == 'g' || type == 'p')){
//        std::cout << "g for move demo or p for ptp demo" << std::endl;
//        cin >> type;
//    }

   std::cout << "Enter 8 values as a move command. Enter d for don't care. Enter quit to go out of the loop" << std::endl;
    while (1){
        if (cnt <8){

            cin >> str;
            try{
                command.data.at(cnt)= stod(str);

            }
            catch(const exception&){
                if (str=="quit")
                    break;
                command.data.at(cnt)= DONTCARE;
            }
        cnt++;
        }
        else{
//            if (type == 'g'){
                sendCommands=true;
                std::cout << "press any key to stop moving" << std::endl;
                cin.clear();
                cin.ignore();
                cout << command << endl;
                getchar();
                sendCommands=false;
////            } else {
//                 pubPtp.publish(command);
//                 cout << command << endl;
//            }

            cnt=0;
            std::cout << "Enter 8 values as a move command. Enter d for don't care. Enter quit to go out of the loop" << std::endl;
        }
    }



    ros::shutdown();
    workThread->interrupt();
    workThread->join();
    delete workThread;

    return 0;
    
}

void doWork(){
    ros::Rate myRate(80);
    while(ros::ok()){
       if (sendCommands){
            pubMove.publish(command);

            cout << "sending " << endl;
        }
    ros::spinOnce();
    myRate.sleep();
    }
}
