#include "Arduino.h"
#include "Suncomm.h"

SunComm Suncomm;

ISR(ADC_vect) {
    Suncomm.isrCallback();
}

SunComm::SunComm() {
    aVal_1 = 0;
    aVal_2 = 0;
    aVal = 0;

    tracker = 0;
    index = 0;

    pulseWidthFlag = STOP;
    blankWidthFlag = STOP;
    pulseWidthTicks = 0;
    blankWidthTicks = 0;

    minVal = 0;
    maxVal = 0;
    lowTH = 0;
    highTH = 0;

    ac = 0;
    ac_1 = 0;

    state = LOW;
    prev = LOW;
    curr = LOW;

    buffer = 0;
    rx_byte = 0;

    rx_bit = 0;
    bit_ctr = 0;

    catchByte = false;

    gotByte = false;
    checkSignal = false;
    doCalculate = false;
    signalPresent = false;

    duty_cycle = 0;
}

SunComm::~SunComm() {

}

void SunComm::attach(int pin) {
    analogPin = pin;
    if(analogPin > 8)
        return;
    
    ADMUX &= B11110000 | analogPin;

    ADMUX &= B11011111;
    ADMUX |= B11000000; //1.1Vref
    ADCSRA |= B10000000;
    ADCSRA |= B00100000;
    ADCSRB &= B11111000;
    ADCSRA &= B11111000;
    ADCSRA |= B00000110;
    ADCSRA |= B00001000;
    sei();

    ADCSRA |= B01000000;

}

bool SunComm::available() {
    float temp = 0;
    
    if(checkSignal) {
        for(int i = 0; i < NUM_OF_SAMPLES; i++) {
            if(val[i] > temp) {
                maxVal = val[i];
                temp = maxVal;
            }
        }

        temp = 0;

        for(int i =0; i < NUM_OF_SAMPLES; i++) {
            if(val[i] < temp) {
                minVal = val[i];
                temp = minVal;
            }
        }

        lowTH = (maxVal - minVal) * 7.0 / 24.0 + minVal;
        highTH = (maxVal - minVal) * 17.0 / 24.0 + minVal;
        index = 0;

        signalPresent = abs(maxVal - minVal) > 50 ? true : false;
    }

    return gotByte;
}

unsigned char SunComm::read() {
    gotByte = false;
    return rx_byte;
}

void SunComm::isrCallback() {
    aVal_1 = aVal;
    aVal = ADCL | (ADCH << 8);
    ac = aVal - aVal_1 + 0.9 * ac_1;
    ac_1 = ac;

    if(index < NUM_OF_SAMPLES) {
        checkSignal = false;
        if(tracker % 7 == 0) {
            val[index] = ac;
            index++;
            tracker = 0;
        }
        tracker++;
    }
    else
        checkSignal = true;

    if(signalPresent) {
        if(state == LOW) {
            if(ac > highTH)
                state = HIGH;
        }
        else if(state == HIGH) {
            if(ac < lowTH)
                state = LOW;
        }

        curr = state;

        if(prev == LOW && curr == HIGH) {
            pulseWidthFlag = START;
            if(blankWidthFlag == START) {
                blankWidthFlag = STOP;
                doCalculate = true;
            }
        }
        else if(prev == HIGH && curr == LOW) {
            pulseWidthFlag = STOP;
            blankWidthFlag = START;
        }

        if(doCalculate) {
#ifdef DEBUG
            pulse_ticks = pulseWidthTicks; 
            blank_ticks = blankWidthTicks;
#endif
            duty_cycle = (float) pulseWidthTicks / ((float) pulseWidthTicks + (float) blankWidthTicks);

            if(duty_cycle > 0.15 && duty_cycle < 0.25)
                rx_bit = STOP_BIT;
            else if(duty_cycle > 0.35 && duty_cycle < 0.45)
                rx_bit = START_BIT;
            else if(duty_cycle > 0.55 && duty_cycle < 0.65)
                rx_bit = LOW_BIT;
            else if(duty_cycle > 0.75 && duty_cycle < 0.85)
                rx_bit = HIGH_BIT;

            if(rx_bit == START_BIT) {
                catchByte = true;
#ifdef DEBUG
                t1 = micros();
#endif
            }
            else {
                if(catchByte) {
                    switch(rx_bit) {
                        case HIGH_BIT   :   buffer = (buffer << 1) | 1;
                                            bit_ctr++;
                                            break;
                        case LOW_BIT    :   buffer = buffer << 1;
                                            bit_ctr++;
                                            break;
                        case STOP_BIT   :   if(bit_ctr == 8) {
                                                rx_byte = buffer;
                                                buffer = 0;
                                                gotByte = true;
                                                bit_ctr = 0;
                                                catchByte = false;
#ifdef DEBUG
                                                t2 = micros();
                                                _diff = t2 - t1;
                                                diff = _diff;
                                                _diff = 0;
                                                t1 = 0;
                                                t2 = 0;
#endif 
                                            }
                                            else {
                                                buffer = 0;
                                                bit_ctr = 0;
                                                catchByte = false;
                                            }
                        default         :   buffer = 0;
                                            bit_ctr = 0;
                                            catchByte = false;
                    }
                }
            }

            pulseWidthTicks = 0;
            blankWidthTicks = 0;

            doCalculate = false;
        }

        if(pulseWidthFlag == START)
            pulseWidthTicks++;

        if(blankWidthFlag == START)
            blankWidthTicks++;

        prev = curr;
    }
    else {
        prev = 0;
        curr = 0;
        gotByte = false;
    }
}
