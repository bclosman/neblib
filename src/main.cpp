/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       closm                                                     */
/*    Created:      1/21/2025, 4:49:38 PM                                     */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/
#include "vex.h"
#include "neblib/timer.hpp"

using namespace vex;

// A global instance of vex::brain used for printing to the V5 brain screen
vex::brain       Brain;

// define your global instances of motors and other devices here


int main() {
   
    while(1) {

        Brain.Screen.clearScreen();

        Brain.Screen.setCursor(1, 1);
        Brain.Screen.print("Brain Time: ");
        Brain.Screen.print(Brain.Timer.value());

        Brain.Screen.setCursor(2, 1);
        Brain.Screen.print("Chrono Time: ");
        Brain.Screen.print(neblib::Timer::getTime(neblib::TimeUnits::second));

        // Allow other tasks to run
        this_thread::sleep_for(100);
    }
}
