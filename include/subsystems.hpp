#pragma once

#include "EZ-Template/api.hpp"
#include "api.h"

extern Drive chassis;

// Your motors, sensors, etc. should go here.  Below are examples

// inline pros::Motor intake(1);
// inline pros::adi::DigitalIn limit_switch('A');

inline pros::MotorGroup chainbar_motors({-1,10});

inline pros::MotorGroup liftbar_motors({2,-9});

inline ez::Piston piston('A');


