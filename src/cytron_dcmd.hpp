#pragma once

#include <Arduino.h>
#include <stdint.h>

class CytronDCMD {
  public:
    CytronDCMD(int pwmPIN, int dirPIN, int channelNumber){
      Ppwm = pwmPIN;
      Pdir = dirPIN;
      Channel = channelNumber;
    };

    void begin(){
      ledcSetup(Channel, 20000, 8);
      ledcAttachPin(Ppwm, Channel);
      pinMode(Pdir, OUTPUT);
    }

    int pmap(int percent){
      percent = constrain(percent, -100, 100);
      return (percent * 255) / 100;
    }

    void run(int percent){
      Outper = pmap(percent);
      digitalWrite(Pdir, Outper>0?HIGH:LOW);
      ledcWrite(Channel, abs(Outper));
    };

    void stop(){
      ledcWrite(Channel, 0);
    }

  private:
    int Ppwm;
    int Pdir;
    int Outper;
    int Channel;
};

