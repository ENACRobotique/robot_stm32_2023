#ifndef ENCODER_H 
#define ENCODER_H


class Encoder{
    public:
        Encoder(int pinA, int pinB): pinA(pinA), pinB(pinB){}
        
        int get_value();
        void init();

    private:
        int pinA;
        int pinB;

        int counter;
};

#endif //ENCODER_H