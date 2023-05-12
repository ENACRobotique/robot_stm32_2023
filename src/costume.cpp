#include "costume.h"

void Deguisement::init(DynamixelSerial* ax, int num){
    this->ax12=ax;
    this->numAX=num;
    this->retract();
}

void Deguisement::retract(){
    this->ax12->moveSpeed(this->numAX,RANGE_C, 100);
}

void Deguisement::deploie(){
    this->ax12->moveSpeed(this->numAX,DEPLOYE_C, 100);
}