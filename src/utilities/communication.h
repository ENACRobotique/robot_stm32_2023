#pragma once

class SerialRadio{
    public:
        void init(auto serial);
        void update();
        void send();

    private:
        char buffer[256];
};

extern SerialRadio comm;