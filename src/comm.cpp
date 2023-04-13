#include "comm.h"

#include <Arduino.h>
#include "AX12A.h"
#include "DisplayController.h"
#include "config.h"
#include "holo_control.h" 


const int LEN_MESSAGES[] = {
    12,// TYPE_POS,
    12,// TYPE_RESET_POS,
    0, // TYPE_STOP,
    0, // TYPE_SLOW,
};

// recap des messages en entrée:
// v <int> <int>: commande de vitesse <linéaire * 1000> <omega * 1000>
// s : arrêt du robot
// a <id> <int> : ordre à un actionneur. id est deux caractères,
//       le premier donnant le type d'actionneur
//                      (a pour AX12A, p pour pompe, e pour electroVanne, s pour servo, d pour le display)
//       le deuxième est un chiffre d'identification.
// d : demande de description des actionneurs. Le robot répond une seule fois
// k: reset l'entier sentDescr
// g [o/v] <int> <int> : changement des valeurs des PIDs (kp et ki)

// recap des messages en sortie:
// m <string>
// r <int> <int> <int> <int> <int>: odométrie moteur <x> <y> <théta> <v> <omega>
// f <int> <int> <int> <int> <int>: odométrie libre (roue Folle) <x> <y> <théta> <v> <omega>
// b <string> <int> <int> <int> [R/RW] <string>: déclaration d'un actionneur (RW) ou d'un capteur (R).
// c <string> <int> : retour de capteur

// Analyse des informations contenues dans les messages SerialCom
void Comm::cmdStop(){// Stops the robot
        holo_control.stop();
        SerialCom.println("m Stopping robot.");
}
void Comm::resetPosition(){
    char *x_addr,*y_addr,*theta_addr;
    uint16_t x,y,theta;
    x_addr = buffer + 1;
    y_addr = buffer +3;
    theta_addr = buffer +5;
    x = *x_addr;
    y = *y_addr;
    theta = *theta_addr;
    
}  
void Comm::cmdActionneurDisplay(){
    int val = -1;
    int params = sscanf(buffer, "a d %d", &val);
    if (params == 1){
        afficheur.setNbDisplayed(val);
    }
}

//Report du démarage

void Comm::reportStart(){
    for (int i=0;i<3;i++){
        SerialCom.println("c TI 42");
    }
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
        default:
            this->etatRadio = IDLE;
            break;
    }
}

void Comm::update()
{    
    switch(this->etatRadio){
    case IDLE:
        char lastRead = 'P';//N'importe quoi sauf un \n pourrait aller, le choix de la lettre P est totalement arbitraire
        while (SerialCom.available() && lastRead != '\n'){
            lastRead = SerialCom.read(); //boucle pour vider le buffer s'il se remplit de caractères à la con
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
            char c = SerialCom.read();
            buffer[0]=c;
            this->setType(c);
        }
        break;
    case WAITING_REST_OF_MESSAGE:
        if (SerialCom.available() >= this->numberOfExpectedBytes){
            uint8_t sum = buffer[0]+this->PROTOCOL_VERSION;
            for (int i=0; i < this->numberOfExpectedBytes; i++){
                buffer[i+1] = SerialCom.read();
                sum += buffer[i+1];
            }
            if (sum == buffer[this->numberOfExpectedBytes]){
                this->execCommand();
            }
            this->etatRadio=IDLE;
        }
        break;
    }
}