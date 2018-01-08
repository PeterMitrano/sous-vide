#pragma once

#include <Arduino.h>

class EventHandler {
  public:
    EventHandler();
    void Dispatch(String event);
    void Run(unsigned long now);

  private:
    bool up_sw1_;
    unsigned int up_sw1_count_;
    bool down_sw1_;
    unsigned int down_sw2_count_;
};
