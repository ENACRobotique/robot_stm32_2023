#ifndef COMM_H
#define COMM_H

#include <Arduino.h>
// Communication Série du robot
typedef enum {
    IDLE,                            // wait \n
    BETWEEN_START_BYTES,             // wait \n
    WAITING_TYPE,                    // read type, deduct size of message
    WAITING_REST_OF_MESSAGE,         // wait for all bytes available, then check and parse
    WAITING_FOR_END_OF_MESSAGE_STRING// read all bytes until \n    NO USE FOUND IN THE DIRECTION HIGH_LEVEL -> LOW_LEVEL
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
    void update();                                      //call that in loop to read messages
    void reportStart();                                 //sends Start Of Match signal to raspy
    void reportActionFinsihed(uint8_t actionNumber);    //sends message indicating that an action ended
    void reportPosition();                              //sends position (x, y, theta) to raspy
    void reportSpeed();                                 //sends speeds (Vx, Vy, Vtheta) to raspy
    void sendMessage (char* message, int size);         //send debug Message

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
    void cmdPos();
    void resetPosition();
    void cmdStop();
    void cmdSlow();
    void cmdclaw();
    void cmdGrab();
    void cmdDrop();
    void cmdTurbine();
    void cmdCostume();
    void cmdSlide();
    void cmdScore();
    void cmdResume();
};
extern Comm radio;
#endif