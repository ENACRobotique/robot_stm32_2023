#include "comm.h"

#include <Arduino.h>
#include "AX12A.h"
#include "DisplayController.h"
#include "config.h"
#include "holo_control.h"
#include "odometry.h" 

// Analyse des informations contenues dans les messages SerialCom
void Comm::cmdStop(){// Stops the robot
        holo_control.stop();
        SerialCom.println("M Stopping robot.");
}

// Recale le robot sur la postion déduite du lidar
void Comm::resetPosition(){
    char *x_addr,*y_addr,*theta_addr;
    float x,y,theta;
    x_addr = buffer + 1;
    y_addr = buffer + 5;
    theta_addr = buffer + 9;
    x = *x_addr;
    y = *y_addr;
    theta = *theta_addr;
    odom.set_x(x);
    odom.set_y(y);
    odom.set_theta(theta);   
}

//Ordre de position
void Comm::cmdPos(){
    char *x_addr,*y_addr,*theta_addr;
    float x,y,theta;
    x_addr = buffer + 1;
    y_addr = buffer + 5;
    theta_addr = buffer + 9;
    x = *x_addr;
    y = *y_addr;
    theta = *theta_addr;
    holo_control.set_ptarget(x, y, theta);
}

// Affiche nombre donné dans message
void Comm::cmdScore(){
    afficheur.setNbDisplayed(*((uint8_t*)(buffer+1)));
}

//Send arbitrary string for debug purposes
void Comm::sendMessage (char const* message, size_t size){
    SerialCom.write("\n\nM",3);
    SerialCom.write(message, size);
    SerialCom.write("\n",1);
}

//Terminé ordre trieuse
void Comm::reportActionFinsihed(uint8_t actionNumber){
    char message[] = "\n\nd**";//start + type + number + checkSum
    message [3] = actionNumber;//parametre du message
    message [4] = message[2] + actionNumber + this->PROTOCOL_VERSION;//calcul du checksum
    SerialCom.write(message,5);
}

//Report du démarage
void Comm::reportStart(){
    char message[] = "\n\nT*";//start + type + checkSum
    message[3] = message[2] + this->PROTOCOL_VERSION;//calcul du checksum
    for (int i=0;i<2;i++){
        SerialCom.write(message,4);//doublé pour redondance
    }
    SerialCom.println("\n\nM Début match !");
}

//Position Report
void Comm::reportPosition(){
    char message[] = "\n\np*************";
    uint8_t sum='p';
    float *addrX = (float *)(message+3);
    float *addrY = (float *)(message+7);
    float *addrTheta = (float *)(message+11);
    *addrX = odom.get_x();
    *addrY = odom.get_y();
    *addrTheta = odom.get_theta();
    for (int i=3; i<15;i++){
        sum+=message[i];
    }
    message[15]=sum;
    SerialCom.write(message,16);
}

//Speed Report
void Comm::reportSpeed(){
    char message[] = "\n\nv*************";
    uint8_t sum='v';
    float *addrVX = (float *)(message+3);
    float *addrVY = (float *)(message+7);
    float *addrVTheta = (float *)(message+11);
    *addrVX = odom.get_vx();
    *addrVY = odom.get_vy();
    *addrVTheta = odom.get_vtheta();
    for (int i=3; i<15;i++){
        sum+=message[i];
    }
    message[15]=sum;
    SerialCom.write(message,16);
}

void Comm::setType(char c){
    switch (c){
        case TYPE_POS:
            this->typeReception = TYPE_POS;
            this->etatRadio = WAITING_REST_OF_MESSAGE;
            this->numberOfExpectedBytes = 13;//3 floats(4 octets) (x, y, theta) + 1 checksum
            break;
        case TYPE_RESET_POS:
            this->typeReception = TYPE_RESET_POS;
            this->etatRadio = WAITING_REST_OF_MESSAGE;
            this->numberOfExpectedBytes = 13;//3 floats(4 octets) (x, y, theta) + 1 checksum
            break;
        case TYPE_STOP:
            this->typeReception = TYPE_STOP;
            this->etatRadio = WAITING_REST_OF_MESSAGE;
            this->numberOfExpectedBytes = 1;//1 checksum
            break;
        case TYPE_SLOW:
            this->typeReception = TYPE_SLOW;
            this->etatRadio = WAITING_REST_OF_MESSAGE;
            this->numberOfExpectedBytes = 1;//1 checksum
            break;
        case TYPE_CLAWS:
            this->typeReception = TYPE_CLAWS;
            this->etatRadio = WAITING_REST_OF_MESSAGE;
            this->numberOfExpectedBytes = 2;//1 char + 1 checksum
            break;
        case TYPE_GRAB_DISKS:
            this->typeReception = TYPE_GRAB_DISKS;
            this->etatRadio = WAITING_REST_OF_MESSAGE;
            this->numberOfExpectedBytes = 3;//2 char (PosPlateau, numéroOrdre) + 1 checksum
            break;
        case TYPE_DROP_DISK:
            this->typeReception = TYPE_DROP_DISK;
            this->etatRadio = WAITING_REST_OF_MESSAGE;
            this->numberOfExpectedBytes = 3;//2 char (PosPlateau, numéroOrdre) + 1 checksum
            break;
        case TYPE_TURBINE:
            this->typeReception = TYPE_TURBINE;
            this->etatRadio = WAITING_REST_OF_MESSAGE;
            this->numberOfExpectedBytes = 2;//1 char + 1 checksum
            break;
        case TYPE_ACTIVATE_COSTUME:
            this->typeReception = TYPE_ACTIVATE_COSTUME;
            this->etatRadio = WAITING_REST_OF_MESSAGE;
            this->numberOfExpectedBytes = 1;//1 checksum
            break;
        case TYPE_DROP_CHERRY_SLIDE:
            this->typeReception = TYPE_DROP_CHERRY_SLIDE;
            this->etatRadio = WAITING_REST_OF_MESSAGE;
            this->numberOfExpectedBytes = 2;//1 char + 1 checksum
            break;
        case TYPE_DISPLAY_POINTS:
            this->typeReception = TYPE_DISPLAY_POINTS;
            this->etatRadio = WAITING_REST_OF_MESSAGE;
            this->numberOfExpectedBytes = 2;//1 char + 1 checksum
            break;
        case TYPE_RESUME:
            this->typeReception = TYPE_RESUME;
            this->etatRadio = WAITING_REST_OF_MESSAGE;
            this->numberOfExpectedBytes = 1;//1 checksum
            break;
        case TYPE_MESSAGE_STR:
            this->etatRadio = WAITING_FOR_END_OF_MESSAGE_STRING;
            break;
        case '\n'://Pour cas improbables avec plus de deux bits de start reçus
            break;
        default:
            this->etatRadio = IDLE;
            break;
    }
}

//executes valid received order that is in buffer
void Comm::execCommand(){
    switch(typeReception){
        case TYPE_POS:
            this->cmdPos();
            break;
        case TYPE_RESET_POS:
            this->resetPosition();
            break;
        case TYPE_STOP:
            this->cmdStop();
            break;
        case TYPE_SLOW:
            break;
        case TYPE_CLAWS:
            break;
        case TYPE_GRAB_DISKS:
            break;
        case TYPE_DROP_DISK:
            break;
        case TYPE_TURBINE:
            break;
        case TYPE_ACTIVATE_COSTUME:
            break;
        case TYPE_DROP_CHERRY_SLIDE:
            break;
        case TYPE_DISPLAY_POINTS:
            this->cmdScore();
            break;
        case TYPE_RESUME:
            break;
    }
}

void Comm::update()
{    
    char lastRead = 'P';//N'importe quoi sauf un \n pourrait aller, le choix de la lettre P est totalement arbitraire
    char c;
    uint8_t sum;
    switch(this->etatRadio){
        case IDLE:
            while (SerialCom.available() && lastRead != '\n'){
                lastRead = SerialCom.read(); //boucle pour vider le buffer du Serial s'il se remplit de caractères à la con
            }
            if(lastRead == '\n'){
                this->etatRadio = BETWEEN_START_BYTES;
            }
            break;
        case BETWEEN_START_BYTES:
            if (SerialCom.available()){
                this->etatRadio = (SerialCom.read() == '\n') ? WAITING_TYPE : IDLE;
            }
            break;
        case WAITING_TYPE:
            if (SerialCom.available()){
                c = SerialCom.read();
                buffer[0]=c;
                this->setType(c);
            }
            break;
        case WAITING_REST_OF_MESSAGE:
            if (SerialCom.available() >= this->numberOfExpectedBytes){
                sum = buffer[0]+this->PROTOCOL_VERSION;
                for (int i=1; i < this->numberOfExpectedBytes; i++){
                    buffer[i] = SerialCom.read();
                    sum += buffer[i];
                }
                if (sum == SerialCom.read()){
                    this->execCommand();
                }
                else {
                    Serial.println("\n\nMChecksum error!");
                }
                this->etatRadio=IDLE;
            }
            break;
        
        case WAITING_FOR_END_OF_MESSAGE_STRING:
            while (SerialCom.available() && lastRead != '\n'){
                lastRead = SerialCom.read(); //boucle pour vider le buffer du Serial, 
                //le bas niveau ne prends pas en compte les messages textuels du haut niveau
            }
            if(lastRead == '\n'){
                this->etatRadio = IDLE;
            }
            break;
    }
}