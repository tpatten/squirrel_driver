#ifndef SENSING_DRIVERS
#define SENSING_DRIVERS



class Driver{   //abstract class: everything in common across sensors is here
//nothing here (no private data)
protected:

    //config matrix (filename?)

    virtual bool setup();   //assuming setup is equal for 2 sensros on 3

public:
    virtual double** read()=0;  //this function reads the data from the sensors and returns a c-matrix (double[][])

};

//------------------------------


class Tactile : public Driver{

public:
    virtual double** read();

};

class Proximity : public Driver{

public:
    virtual double** read();

};

class Wrist : public Driver{

public:
    virtual double** read();

};

#endif
