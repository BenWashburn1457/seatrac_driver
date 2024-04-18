#include <iostream>
#include <stdio.h>

#include <seatrac_driver/Calibration.h>
using namespace narval::seatrac;

int main(int argc, char *argv[])
{
    calibration::calibrateAccelerometer();
    calibration::calibrateMagnetometer();
    return 0;
}
