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
        SerialCom.println("\n\nM Stopping robot.");
}

// Recale le robot sur la postion déduite du lidar
void Comm::resetPosition(){

    struct msgPos *msg = reinterpret_cast<struct msgPos *>(MSG_BUF);

    odom.set_x(msg->x);
    odom.set_y(msg->y);
    odom.set_theta(msg->theta);
    SerialCom.println("\n\nM Robot position reseted.");
}

//Ordre de position
void Comm::cmdPos(){
    struct msgPos *msg = reinterpret_cast<struct msgPos *>(MSG_BUF);
    holo_control.set_ptarget(msg->x, msg->y, msg->theta);
    SerialCom.println("\n\nMPos order acknowleged");

}

// Affiche nombre donné dans message
void Comm::cmdScore(){
    afficheur.setNbDisplayed(buffer[1]);
    //SerialCom.print("\n\nMAfficheur affiche ");
    //SerialCom.println(*((uint8_t*)(buffer+1)));

    char txt[20];
    int len = snprintf(txt, 20, "afficheur, %d", buffer[1]);
    sendMessage(txt, len);
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
    uint8_t sum='p'+this->PROTOCOL_VERSION;

    struct msgPos pos = {
        .x = odom.get_x(),
        .y = odom.get_y(),
        .theta = odom.get_theta(),
    };
    memcpy(&message[3], &pos, sizeof(struct msgPos));

    // float x = odom.get_x();
    // float y = odom.get_y();
    // float theta = odom.get_theta();
    // memcpy(&message[3], &x, sizeof(float));
    // memcpy(&message[7], &y, sizeof(float));
    // memcpy(&message[11], &theta, sizeof(float));

    for (int i=3; i<15;i++){
        sum+=message[i];
    }
    message[15]=sum;
    SerialCom.write(message,16);
}

//Speed Report
void Comm::reportSpeed(){
    char message[] = "\n\nv*************";
    uint8_t sum='v'+this->PROTOCOL_VERSION;
    struct msgPos speed = {
        .x = static_cast<float>(odom.get_vx()),
        .y = static_cast<float>(odom.get_vy()),
        .theta = static_cast<float>(odom.get_vtheta()),
    };
    memcpy(&message[3], &speed, sizeof(struct msgPos));
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
            this->cmdSlow();
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
            this->cmdResume();
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

void Comm::cmdSlow(){
    //float *factor = reinterpret_cast<float*>(MSG_BUF);
    holo_control.set_ratio_slow(2.f);
}

void Comm::cmdResume()
{
    holo_control.set_ratio_slow(1.f);
}