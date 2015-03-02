#include <string>

#include "../include/squirrel_sensing_node/node.h"

using namespace std;

int main(int argc,char** argv){

    string name="sensing";  //can be read from argv

    ros::init(argc, argv, name);

    cout << "Creating " << name << " node " << endl;

    SensingNode sensing(name);

    cout << "Executing " << name << " node " << endl;

    sensing.run();

    cout << name << " node has terminated his execution" << endl;
}
