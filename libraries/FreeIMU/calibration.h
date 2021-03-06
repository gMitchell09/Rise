
/**
 * FreeIMU calibration header. Automatically generated by octave AccMagnCalib.m.
 * Do not edit manually unless you know what you are doing.
*/

/* // following example of calibration.h 
#define CALIBRATION_H

const int acc_off_x = 205;
const int acc_off_y = -39;
const int acc_off_z = 1063;
const float acc_scale_x = 7948.565970;
const float acc_scale_y = 8305.469320;
const float acc_scale_z = 8486.650841;

const int magn_off_x = 67;
const int magn_off_y = -59;
const int magn_off_z = 26;
const float magn_scale_x = 527.652115;
const float magn_scale_y = 569.016790;
const float magn_scale_z = 514.710857;
*/

#ifndef CALIBRATION_H
#define CALIBRATION_H

const int acc_off_x = 0;
const int acc_off_y = 0;
const int acc_off_z = 0;
const float acc_scale_x = 16384;
const float acc_scale_y = 16384;
const float acc_scale_z = 16384;


const int magn_off_x = 0;
const int magn_off_y = 0;
const int magn_off_z = 0;
const float magn_scale_x = 440;
const float magn_scale_y = 440;
const float magn_scale_z = 440;

#endif