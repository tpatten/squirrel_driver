#include <stdio.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <unistd.h>
#include <string>
#include <iostream>
#include <stdlib.h>
#include <fstream>
#include <sstream>
#include <cmath>
#include <ros/package.h>
#include <ros/ros.h>


#include "../include/squirrel_sensing_node/sensing_drivers.h"

//#define TEST    //undefine to remove testing code

using namespace std;

//makes the connection with m_portname
bool Driver::setup(){    //assuming setup is equal for 2 sensros on 3

    m_fileDesc=open(m_portname.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
    fcntl(m_fileDesc, F_SETFL,0);//reset file status flags

    /* Set up the control structure */
     struct termios toptions;
     /* Get currently set options for the tty */
     tcgetattr(m_fileDesc, &toptions);
     /* 9600 baud */
      cfsetispeed(&toptions, B9600);
      cfsetospeed(&toptions, B9600);
      /* 8 bits, no parity, no stop bits */
       toptions.c_cflag &= ~PARENB;
       toptions.c_cflag &= ~CSTOPB;
       toptions.c_cflag &= ~CSIZE;
       toptions.c_cflag |= CS8; //should be one stop bit
       /* Canonical mode */
        toptions.c_lflag |= ICANON;

        //---------------------------------------------------------------
        /*non canonical mode */
       // /* no hardware flow control */
       //toptions.c_cflag &= ~CRTSCTS;
       // /* enable receiver, ignore status lines */
       //toptions.c_cflag |= CREAD | CLOCAL;
       // /* disable input/output flow control, disable restart chars */
       //toptions.c_iflag &= ~(IXON | IXOFF | IXANY);
       // /* disable canonical input, disable echo,
       //disable visually erase chars,
       //disable terminal-generated signals */
       //toptions.c_iflag &= ~(ICANON | ECHO | ECHOE | ISIG);
       // /* disable output processing */
       //toptions.c_oflag &= ~OPOST;
        //---------------------------------------------------------------

    /* commit the options */
     tcsetattr(m_fileDesc, TCSANOW, &toptions);
     /* Flush anything already in the serial buffer */
     tcflush(m_fileDesc, TCIFLUSH);

     return true;
}

void Driver::flush(){
    /* Flush anything already in the serial buffer */
    tcflush(m_fileDesc, TCIFLUSH);
}

//------------------------------TACTILE and PROXIMITY

const int Tactile::NUM_TACT=9; //number of tactile values
const int Tactile::NUM_PROX=6;
const int Tactile::NUM_VALS=Tactile::NUM_TACT+Tactile::NUM_PROX;  //should be 15

const int Tactile::NUM_HISTORY_VALS=10;
const int Tactile::NUM_BIAS_VALS=100;
const double Tactile::STATIONARY_TACTILE_THREASHOLD=0.00;    //volts
const double Tactile::STATIONARY_PROXIMITY_THREASHOLD=0.02;    //volts

//calibration coefficients tactile sensor
const double Tactile::A11_TACT=26.25;
const double Tactile::A12_TACT=21.96;
const double Tactile::A13_TACT=2.608;

const double Tactile::A21_TACT=-2.152;
const double Tactile::A22_TACT=-1.381;
const double Tactile::A23_TACT=0;

const double Tactile::A31_TACT=-12.65;
const double Tactile::A32_TACT=0;
const double Tactile::A33_TACT=27.52;

//maximums of calibration curves in volts for tactile //NEVER set them to 0 or you will get a NaN
const double Tactile::MAX1_V1=0.35;
const double Tactile::MAX1_V2=0.42;
const double Tactile::MAX1_V3=3.56;

const double Tactile::MAX2_V1=0.18;
const double Tactile::MAX2_V2=0.25;
const double Tactile::MAX2_V3=1;

const double Tactile::MAX3_V1=0.15;
const double Tactile::MAX3_V3=-0.12;

//calibration coefficients proximity sensor
const double Tactile::A_PROX=20.74;
const double Tactile::B_PROX=-0.1808; //exponent
const double Tactile::C_PROX=-17.28;

//calibration maximum for proximity
const double Tactile::MAX_PROX=2.5;

//number of values used to calculate average flattening value
const int Tactile::NUM_FLATTENING_TORQUES=10;



Tactile::Tactile(const std::string& portname){

    m_portname=portname;
    m_accumulator_t1=0;
    m_accumulator_t2=0;
    m_divider=1;
    for(int i=0;i<(3*2);++i)
    {
        torque_perc.push_back(0.0); //3 axis x 2 values set to 0
    }

    //initialising history
    for(int i=0;i<NUM_TACT;i++)
    {
        history_val_tact.push_back(0);
        history_tact.push_back(new queue<double>());
        mean.push_back(vector<double>() );
    }
    for(int i=0;i<NUM_PROX;i++)
    {
        history_val_prox.push_back(0);
        history_prox.push_back(new queue<double>());
        mean.push_back(vector<double>() );
    }


    //read from conf.ini the divider values
    string filepath=ros::package::getPath("squirrel_sensing_node");
    filepath+="/tactile_calibration.ini";
    ifstream config(filepath.c_str());
    bool isReadMaximums=false;  //if more items will be added to the ini file, this will became an enum
    if(config.good()){
        string line;
        int sensId=0;
        while(getline(config,line) && sensId<=NUM_VALS){    //one divider per sensor reading

            if(line.at(0)=='@') //this caracther changes the parsing
            {
              isReadMaximums=true;
            }

            if(!isReadMaximums && line.length()!=0 && line.at(0)!='#' && line.at(0)!='@'){ //if line is not a comment or blank
                istringstream iss(line);
                double val;
                iss >> val;                         //parsing dividers
                divider.push_back(val);
                sensId++;
            }
            else if(isReadMaximums && line.length()!=0 && line.at(0)!='#' && line.at(0)!='@'){ //if line is not a comment or blank
                istringstream iss(line);
                double val;                         //parsing maximums
                iss >> val;
                maximumTorque.push_back(val);
            }
        }

    }else{
        ROS_INFO("ERROR: Could not locate file config.ini, assumed no dividers (==1)");
        for(int i=0;i<NUM_TACT;++i){
            divider.push_back(1);
        }
    }

    setup();
}

Tactile::~Tactile(){

    close(m_fileDesc);

    for(int i=0;i<NUM_TACT;i++)
    {
        delete history_tact[i];
    }

    for(int i=0;i<NUM_PROX;i++)
    {

        delete history_prox[i];
    }
}

//returns true if data is stationary, input is the value and the number of sensor used
bool Tactile::isStationaryTact(const double val,const int idx){

    history_tact.at(idx)->push(val);
    history_val_tact.at(idx)+=val;
    if(history_tact.at(idx)->size()<NUM_HISTORY_VALS){ //if we have not enough values yet
        return false;
    }

    history_val_tact.at(idx)-=history_tact.at(idx)->front(); //subtract oldest value
    history_tact.at(idx)->pop();
    return ( (history_val_tact.at(idx)/NUM_HISTORY_VALS) < STATIONARY_TACTILE_THREASHOLD);

}

//returns true if data is stationary, input is the value and the sensor id
//WARNING as it is now, only the first value in a set of 3 is checked for stationarity
bool Tactile::isStationaryProx(const double val,const int idx){

    history_prox.at(idx)->push(val);
    history_val_prox.at(idx)+=val;
    if(history_prox.at(idx)->size()<NUM_HISTORY_VALS){ //if we have not enough values yet
        return false;
    }

    history_val_prox.at(idx)-=history_prox.at(idx)->front(); //subtract oldest value
    history_prox.at(idx)->pop();
    return ( (history_val_prox.at(idx)/NUM_HISTORY_VALS) < STATIONARY_PROXIMITY_THREASHOLD);

}

//calculates mean of first 10 values and return the bias value
double Tactile::bias(const int idx,const double val)
{
    if(mean[idx].size()==0){
        mean[idx].push_back(val);
        return 1.0;
    }
    //else calculate mean of past values
    if(mean[idx].size()<NUM_BIAS_VALS){
        mean[idx].push_back(val);
    }

    double accumulator=0.0;

    for(int i=0;i<mean[idx].size();++i){
        accumulator+=mean[idx].at(i);
    }

    return (accumulator/mean[idx].size());
}

void Tactile::flatteningProcessing(vector<double>& reads,const int idx)
{
    if(m_divider==NUM_FLATTENING_TORQUES)
    {
        flattenTorque(reads,idx);      //flattens torque values
        return;
    }

    m_accumulator_t1+=reads[idx+1];   // 3 is number of values per axis
    m_accumulator_t2+=reads[idx+2];   // +1 and +2 are to select torque 1 and 2

    if(idx==6)  //if we are processing the last triplet (6+2=9 which is the last triplet) then increment the counter
    {
        ++m_divider;
    }
}


std::vector<double>& Tactile::readData(){

    std::vector<double>* res=new vector<double>(NUM_VALS,-1.0); //if a -1 is given it means there was a problem with reading that value

    char buff[255];
#ifndef TEST
    int rd=read(m_fileDesc,buff,255);

#else
    buff[0]='\n';
    cout << "Input volts: " ;
   for(int i=0;i<NUM_VALS;i++)
  {
       res->at(i)=0.1;       //artificial testing volt value
        cout << res->at(i) << " ";
  }
    cout << endl;
   //testing
#endif

    //parsing from arduino
    bool done=false;
    int readingNum=0;
    string st_buf;

    for(int i=0;i<255 && !done;i++){

        if(buff[i]==' '){

            double val=atof(st_buf.c_str());

            if(readingNum==NUM_VALS){
                done = true;                
            }else{

                res->at(readingNum)=val-bias(readingNum,val);  //biasing to zero
            }//if read enough values

            st_buf="";
            readingNum++;


        }//if separator found
        else if(buff[i]=='\n'){
            done = true;            
        }//else if end of line found

        st_buf+=buff[i];

    }//for stuff in buff

    //parsing from arduino - END


    for(int i=0;i<NUM_TACT;i++){//biasing to calibration
        res->at(i)= res->at(i)*divider[i]; //calibartion curve maximum is (5-1)
    }


    //cout << "Tactile vals: ";
    for(int i=0;i<NUM_TACT;i+=3){//first 9 values are tactile


        if(!isStationaryTact(res->at(i),i)){      //if values changed
          convertTact(*res,i);     //we pass 3 values at time (that is why the for increments i+3)
          flatteningProcessing(*res,i);
        }else{
            res->at(i)=0;
        }//if not stationary


        //cout << res->at(i) << " " << res->at(i+1) << " " << res->at(i+2) << " ";
    }
   // cout << endl ;

    calculateTorquePerc(*res);  //fills in the torque perc vector

    for(int i=NUM_TACT;i<NUM_VALS;i++){//biasing to calibration distance
        res->at(i)= res->at(i)*((divider[i])/MAX_PROX); //calibartion curve maximum is accounted
    }


    //cout << "Proximity vals: ";
    for(int i=NUM_TACT;i<NUM_VALS;i++){//last 6 values are proximity

        if(!isStationaryProx(res->at(i),i-NUM_TACT)){      //if values changed
            res->at(i)=convertProx(res->at(i));
        }else{
            res->at(i)=-100; //This is when there is no signal detected
        }

       // cout << res->at(i) << " ";
    }
    //cout << endl << endl;


   // cin.ignore();

    return *res;
}

//convert volts into newtons
void Tactile::convertTact(vector<double>& num,int idx){

    if(num.size() < idx+2)
    {
        cout << "[Tactile::convertTact] not enough sensor readings to parse in range [" << idx << " " << idx+3 << "] which is larger than maximum number of elements " << num.size() << endl;
        cout << "[Tactile::convertTact] WARNING - Values have NOT been processed!" << endl;
        return; //just to avoid crashes, we could assert it as well
    }

    for(int i=0;i<2;i++)
    {
        if(num[idx+i]==-1) {   //if value not valid
            num[idx+i]=-1;
        }
    }

    double v1=num.at(idx);
    double v2=num.at(idx+1);
    double v3=num.at(idx+2);


    if(num[idx]!=-1)    //these checks are done to be sure that illigal values are not used for computing the formulas
    {
        num[idx]=((A11_TACT/MAX1_V1)*v1)+((A12_TACT/MAX1_V2)*v2)+((A13_TACT/MAX1_V3)*v3);
    }
    if(num[idx+1]!=-1)
    {
        num[idx+1]=((A21_TACT/MAX2_V1)*v1)+((A22_TACT/MAX2_V2)*v2)+((A23_TACT/MAX2_V3)*v3);
    }
    if(num[idx+2]!=-1)
    {
        num[idx+2]=((A31_TACT/MAX3_V1)*v1)+((A33_TACT/MAX3_V3)*v3);
    }

}

//flatten torque values using the number initialised at startup
void Tactile::flattenTorque(vector<double>& num,int idx)
{
    double flattening_t1= m_accumulator_t1/m_divider;
    double flattening_t2= m_accumulator_t2/m_divider;

    num.at(idx+1)-=flattening_t1;    //first torque
    num.at(idx+2)-=flattening_t2;    //second torque
}

//convert volts into distance
double Tactile::convertProx(const double num){

    if(num==-1) {   //if value not valid
        return -1;
    }

    double val=num;   //subtract 1 volt

    return (-((A_PROX*pow(val,B_PROX))+C_PROX));


}

void Tactile::calculateTorquePerc(vector<double>& num)
{

  for(int j=0,i=0;j<9;j+=3,i+=2)    //TODO 9 is the first proximity value
  {
      torque_perc[i]=num.at(j+1);
      torque_perc[i+1]=num.at(j+2);
  }


for(int i=0;i<torque_perc.size();++i)
  {
      torque_perc[i]=(torque_perc.at(i)/maximumTorque.at(i))*100;
  }

}

vector<double>& Tactile::readTorquePerc()
{
    return torque_perc;
}

//----------------------WRIST-----------------------

Wrist::Wrist(const std::string& portname)
{

    ft17=new FT17Interface ( portname.c_str() );

    ft17->init();
	// FT17 configured in POLLING mode
    ft17->configure_polling ( (uint16_t)127 );
}

Wrist::~Wrist()
{
    //ft17->stop_broadcast();	//it seems is not needed anymore
    delete ft17;
}

std::vector<double>& Wrist::readData(){

	if(ft17==NULL){

        vector<double>tmp(WristDataNum,0);
        return tmp; //WARNING reference to local variable returned

	}

    // get the FT17 data
    //ft17->get_broadcast_data ( ft_bc_data );
	ft17->get_single_data ( ft_bc_data );	//the data structures should be the same but the values require to be checked

    vector<double>* res=new vector<double>(WristDataNum,0);



    // fill the FT_filt msg
    //frame_id = std::to_string ( ft_bc_data.board_id ) ; //frame ID, if needed

    res->at(Wrist::ForceX) = ft_bc_data.FT_filt[Wrist::ForceX];
    res->at(Wrist::ForceY) = ft_bc_data.FT_filt[Wrist::ForceY];
    res->at(Wrist::ForceZ) = ft_bc_data.FT_filt[Wrist::ForceZ];
    res->at(Wrist::TorqueX) = ft_bc_data.FT_filt[Wrist::TorqueX];
    res->at(Wrist::TorqueY) = ft_bc_data.FT_filt[Wrist::TorqueY];
    res->at(Wrist::TorqueZ) = ft_bc_data.FT_filt[Wrist::TorqueZ];
    res->at(Wrist::Timestamp) =  ft_bc_data.tStamp ;

    return *res;
}

