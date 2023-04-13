#ifndef COMM_H
#define COMM_H

#include <Arduino.h>
// Communication Série du robot
typedef enum {
    IDLE,
    BETWEEN_START_BYTES,
    WAITING_TYPE,
    WAITING_REST_OF_MESSAGE
}radioStates_t;


typedef enum {
    TYPE_POS ='p',
    TYPE_RESET_POS='r',
    TYPE_STOP='S',
    TYPE_SLOW='s',
    TYPE_CLAWS='g',
    TYPE_GRAB_DISKS='a',
    TYPE_DROP_DISK='d',
    TYPE_TURBINE='t',
    TYPE_ACTIVATE_COSTUME='D',
    TYPE_DROP_CHERRY_SLIDE='T',
    TYPE_DISPLAY_POINTS='P',
    TYPE_RESUME='R',
} messageTypes_t;



class Comm
{
public:
    void update();
    void reportStart();

private:
    //Attributs
    //Serial2 for usb, Serial3 for Xbee
    HardwareSerial& SerialCom = Serial2; //Serial3;

    char buffer[20];
    int buf_index = 0;
    radioStates_t etatRadio=IDLE;
    messageTypes_t typeReception;
    int numberOfExpectedBytes;
    const int PROTOCOL_VERSION = 1;

    // méthodes privées
    void setType(char c);
    void execCommand();
    void cmdStop();
    void resetPosition();
    void cmdActionneurDisplay();
    void cmdPos();
    void cmdSlow();
    void cmdclaw();
    void cmdGrab();
    void cmdDrop();
    void cmdTurbine();
    void cmdCostume();
    void cmdSlide();
    void cmdScore();
    void cmdResume();
    void sendMessage(char*,int len);
};
extern Comm radio;
#endif