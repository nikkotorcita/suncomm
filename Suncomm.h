#ifndef _SUNCOMM_
#define _SUNCOMM_

#define NUM_OF_SAMPLES 150

#define WAIT  0
#define START 1
#define STOP  2

#define START_BIT  1
#define STOP_BIT   2
#define LOW_BIT    3
#define HIGH_BIT   4

#define LOW 0
#define HIGH 1

class SunComm
{
    public:
        SunComm();
        ~SunComm();
        void attach(int pin);
        bool available();
        unsigned char read();
        void isrCallback();
        bool ithappened;
    private:
        int analogPin;

        volatile int aVal_1;
        volatile int aVal_2;
        volatile int aVal;

        float val[NUM_OF_SAMPLES];
#ifdef DEBUG
        unsigned long int t1;
        unsigned long int t2;
        unsigned long int diff;
        unsigned long int _diff;
#endif
        int tracker;
        int index;

        int pulseWidthFlag;
        int blankWidthFlag;
        int pulseWidthTicks;
        int blankWidthTicks;
#ifdef DEBUG
        int pulse_ticks;
        int blank_ticks;
        int period;
#endif
        float minVal;
        float maxVal;
        float lowTH;
        float highTH;

        float ac;
        float ac_1;

        int state;
        int prev;
        int curr;

        int buffer;
        int rx_byte;

        int rx_bit;
        int bit_ctr;

        float duty_cycle;

        bool catchByte;
        bool gotByte;
        bool checkSignal;
        bool doCalculate;
        bool signalPresent;

};

extern SunComm Suncomm;
#endif
