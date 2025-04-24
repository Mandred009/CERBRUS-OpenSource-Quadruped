#ifndef QUAD_SERVO_H
#define QUAD_SERVO_H

#include <Arduino.h>
#include <SCServo.h>

extern SMS_STS st;

extern float l1,l2,l3; // Leg Link Parameters

extern byte ID[12];

extern byte ACC[12]; // Servo Acceleration list

extern float constraints[4][3][2][2];


extern int all_leg[4]; // Leg nos as used in code

void ik(float x, float y, float z, float constraint_list[4][3][2][2], float (&theta)[3], int leg_index);

void stable_state(int legs_to_move[], int num_legs,float h1, float h2, float h3, float h4, float z1, float z2, float z3, float z4, float y1, float y2, float y3, float y4, float vel1, float vel2, float vel3, float vel4);

int angle_to_servo_converter(float angle, float servo_map[], float angle_map[]);

void calibration_pos();


#endif
