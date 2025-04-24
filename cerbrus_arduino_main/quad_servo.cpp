// Leg Servos cpp file

#include "quad_servo.h"

SMS_STS st;


float l1 = 58,l2 = 130.5,l3 = 190; // Leg Link Parameters

byte ID[12] = {5, 6, 4, 7, 13, 15, 8, 1, 10, 12, 9, 11}; // Servo IDS

byte ACC[12] = {80,80,80,80,80,80,80,80,80,80,80,80}; // Servo Acceleration list

float constraints[4][3][2][2] = {
    // Leg 1
    {
        {{1006, 3054}, {0, 180}}, // Hip joint servo range and angle range 
        {{3174, 1126}, {90, -90}}, // Knee joint servo range and angle range
        {{2924, 876}, {90, -90}}  // Ankle joint servo range and angle range
    },
    // Leg 2
    {
        {{3104, 1056}, {0, 180}}, // Hip joint servo range and angle range
        {{1024, 3072}, {-90, 90}}, // Knee joint servo range and angle range
        {{3072, 1024}, {-90, 90}}  // Ankle joint servo range and angle range
    },
    // Leg 3
    {
        {{3144, 1048}, {0, 180}}, // Hip joint servo range and angle range
        {{1026, 3074}, {-90, 90}}, // Knee joint servo range and angle range
        {{3274, 1226}, {-90, 90}}  // Ankle joint servo range and angle range
    },
    // Leg 4
    {
        {{1024, 3072}, {0, 180}}, // Hip joint servo range and angle range
        {{3072, 1024}, {90, -90}}, // Knee joint servo range and angle range
        {{3144, 1096}, {-90, 90}}  // Ankle joint servo range and angle range
    }
};


int all_leg[4]={0,1,2,3};



void stable_state(int legs_to_move[], int num_legs,float h1, float h2, float h3, float h4, float z1, float z2, float z3, float z4, float y1, float y2, float y3, float y4, float vel1, float vel2, float vel3, float vel4) {
    // Define the stable X and Z positions

    float ptsx[] = {h1, h2, h3, h4};  // Leg 1, 2, 3, 4 X positions
    float ptsz[] = {z1, z2, z3, z4};  // Leg 1, 2, 3, 4 Z positions
    float ptsy[] = {20+y1, 20+y2, 20+y3, 20+y4}; // +20 to widen the leg base and make side motion possible
    float velocity[] = {vel1, vel2, vel3, vel4};  // Speed for each leg

    u16 Speed_all[num_legs*3];
    float theta_all[num_legs*3];
    float theta[3];

    byte selected_IDs[num_legs*3]; 

    byte acc[num_legs*3]; 

    // Loop through the specified legs to move
    for (int i = 0; i < num_legs; i++) {
        int leg = legs_to_move[i];  // Get the leg index from the input array
        float x = ptsx[leg];
        float z = ptsz[leg];
        float y = ptsy[leg];;

        ik(x, y, z, constraints, theta, leg);  // Calculate inverse kinematics for this leg

        // Store the computed joint angles for the selected leg
        theta_all[i * 3 + 0] = theta[0];  // theta1 for current leg
        theta_all[i * 3 + 1] = theta[1];  // theta2 for current leg
        theta_all[i * 3 + 2] = theta[2];  // theta3 for current leg

        // Set the speed for the selected leg
        Speed_all[i * 3 + 0] = velocity[leg];  // Speed for theta1
        Speed_all[i * 3 + 1] = velocity[leg];  // Speed for theta2
        Speed_all[i * 3 + 2] = velocity[leg];  // Speed for theta3

        selected_IDs[i * 3 + 0] = ID[leg*3+0]; 
        selected_IDs[i * 3 + 1] = ID[leg*3+1];  
        selected_IDs[i * 3 + 2] = ID[leg*3+2];  

        acc[i * 3 + 0] = ACC[leg*3+0];  
        acc[i * 3 + 1] = ACC[leg*3+1];  
        acc[i * 3 + 2] = ACC[leg*3+2];  
    }

    // Convert theta_all (float) to s16 (short int)
    s16 theta_all_s16[num_legs*3];
    for (int i = 0; i < num_legs*3; i++) {
        theta_all_s16[i] = (s16)theta_all[i];  // Convert each angle to short integer
    }

    // Synchronize the servo positions
    st.SyncWritePosEx(selected_IDs, num_legs*3, theta_all_s16, Speed_all, acc); 

    
}


int angle_to_servo_converter(float angle, float servo_map[], float angle_map[]) {
    // Calculate the converted servo position
    float converted_servo = servo_map[0] - ((servo_map[0] - servo_map[1]) * (angle_map[0] - angle) / (angle_map[0] - angle_map[1]));
    return abs((int)converted_servo);
}

// Function to perform inverse kinematics and return theta values as an array
void ik(float x, float y, float z, float constraint_list[4][3][2][2], float (&theta)[3], int leg_index) {
    // Select constraints based on the leg index
    float constraint_1[2] = {constraint_list[leg_index][0][0][0], constraint_list[leg_index][0][0][1]}; // Hip servo range
    float constraint_2[2] = {constraint_list[leg_index][1][0][0], constraint_list[leg_index][1][0][1]}; // Knee servo range
    float constraint_3[2] = {constraint_list[leg_index][2][0][0], constraint_list[leg_index][2][0][1]}; // Ankle servo range

    float angle_map_1[2] = {constraint_list[leg_index][0][1][0], constraint_list[leg_index][0][1][1]}; // Hip angle limits
    float angle_map_2[2] = {constraint_list[leg_index][1][1][0], constraint_list[leg_index][1][1][1]}; // Knee angle limits
    float angle_map_3[2] = {constraint_list[leg_index][2][1][0], constraint_list[leg_index][2][1][1]}; // Ankle angle limits

    // Calculate theta1 (hip joint)
    float theta1 = atan2(y, x);
    theta1 = abs(degrees(theta1)); // Convert to degrees

    // Calculate other parameters for IK
    float s = z;
    float r = (sqrt(x * x + y * y) - l1);
    float D = ((r * r) + (s * s) - (l2 * l2) - (l3 * l3)) / (2 * l2 * l3);

    // Calculate theta3 (ankle joint)
    float theta3 = acos(D);

    // Calculate theta2 (knee joint)
    float theta2 = atan2(s, r) - atan2(l3 * sin(theta3), l2 + (l3 * cos(theta3)));

    // Convert theta2 and theta3 to degrees
    float theta2_d = degrees(theta2);
    float theta3_d = degrees(theta3);

    // Convert angles to servo positions using angle_to_servo_converter
    theta[0] = angle_to_servo_converter(theta1, constraint_1, angle_map_1); // Hip joint
    theta[1] = angle_to_servo_converter(theta2_d, constraint_2, angle_map_2); // Knee joint
    theta[2] = angle_to_servo_converter(theta3_d, constraint_3, angle_map_3); // Ankle joint
}

// Change this function values if your calibration center points are different
void calibration_pos(){
  s16 center_pt[12]={2030,2150,1900,2080,2048,2048,2096,2050,2250,2048,2048,2120};
  u16 Speed[12]={500,500,500,500,500,500,500,500,500,500,500,500};
  st.SyncWritePosEx(ID, 12, center_pt, Speed , ACC); 
}
