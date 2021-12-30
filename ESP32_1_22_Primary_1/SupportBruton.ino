//int S2S_pot;
//int Output_S2S_pwm
//int current_pos_drive;  // variables for smoothing main drive
//int target_pos_drive;
//int pot_drive;   // target position/inout
//int diff_drive; // difference of position
//double easing_drive;
//

//
//double Pk3 = 3; // position it goes to
//double Ik3 = 0;
//double Dk3 = 0;
//double Setpoint3, Input3, Output3, Output_Drive;    // PID variables - Main drive motor
//
//PID PID_Drive(&Input3, &Output3, &Setpoint3, Pk3, Ik3 , Dk3, DIRECT);    // Main drive motor
////
////
////
////***************Main Drive PID***************
//
//
// target_pos_drive = map(ch1, 0,512,65,-65);
//
// easing_drive = 1000;          //modify this value for sensitivity
// easing_drive /= 1000;
//
// // Work out the required travel.
// diff_drive = target_pos_drive - current_pos_drive;    
//
// // Avoid any strange zero condition
// if( diff_drive != 0.00 ) {
//    current_pos_drive += diff_drive * easing_drive;
// }
//
// Setpoint3 = current_pos_drive;
//
// Input3 = pitch+3;
//
// PID3.Compute();
//
// 
// if (Output3 <= 1)                                      // decide which way to turn the wheels based on deadSpot variable
//    {
//    Output3a = abs(Output3);
//    analogWrite(11, Output3a);                                // set PWM pins 
//    analogWrite(12, 0);
//    }
// else if (Output3 > 1)                          // decide which way to turn the wheels based on deadSpot variable
//    { 
//    Output3a = abs(Output3);
//    analogWrite(12, Output3a);  
//    analogWrite(11, 0);
//    }
