// /*
//  * DCM.h
//  *
//  * Created: 01.02.2016 20:49:34
//  *  Author: Metin
//  */ 
// /*
// MinIMU-9-Arduino-AHRS
// Pololu MinIMU-9 + Arduino AHRS (Attitude and Heading Reference System)
// Copyright (c) 2011 Pololu Corporation.
// http://www.pololu.com/
// MinIMU-9-Arduino-AHRS is based on sf9domahrs by Doug Weibel and Jose Julio:
// http://code.google.com/p/sf9domahrs/
// sf9domahrs is based on ArduIMU v1.5 by Jordi Munoz and William Premerlani, Jose
// Julio and Doug Weibel:
// http://code.google.com/p/ardu-imu/
// MinIMU-9-Arduino-AHRS is free software: you can redistribute it and/or modify it
// under the terms of the GNU Lesser General Public License as published by the
// Free Software Foundation, either version 3 of the License, or (at your option)
// any later version.
// MinIMU-9-Arduino-AHRS is distributed in the hope that it will be useful, but
// WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
// FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License for
// more details.
// You should have received a copy of the GNU Lesser General Public License along
// with MinIMU-9-Arduino-AHRS. If not, see <http://www.gnu.org/licenses/>.
// */
// 
// #define GRAVITY 256  //this equivalent to 1G in the raw data coming from the accelerometer
// 
// #define ToRad(x) ((x)*0.01745329252)  // *pi/180
// #define ToDeg(x) ((x)*57.2957795131)  // *180/pi
// 
// #define Gyro_Gain_X 0.07 //X axis Gyro gain
// #define Gyro_Gain_Y 0.07 //Y axis Gyro gain
// #define Gyro_Gain_Z 0.07 //Z axis Gyro gain
// #define Gyro_Scaled_X(x) ((x)*ToRad(Gyro_Gain_X)) //Return the scaled ADC raw data of the gyro in radians for second
// #define Gyro_Scaled_Y(x) ((x)*ToRad(Gyro_Gain_Y)) //Return the scaled ADC raw data of the gyro in radians for second
// #define Gyro_Scaled_Z(x) ((x)*ToRad(Gyro_Gain_Z)) //Return the scaled ADC raw data of the gyro in radians for second
// 
// #define Kp_ROLLPITCH 0.02
// #define Ki_ROLLPITCH 0.00002
// #define Kp_YAW 1.2
// #define Ki_YAW 0.00002
// 
// float G_Dt=0.02;    // Integration time (DCM algorithm)  We will run the integration loop at 50Hz if possible
// 
// long timer=0;   //general purpuse timer
// long timer_old;
// long timer24=0; //Second timer used to print values
// int AN[6]; //array that stores the gyro and accelerometer data
// int AN_OFFSET[6]={0,0,0,0,0,0}; //Array that stores the Offset of the sensors
// 
// int gyro_x;
// int gyro_y;
// int gyro_z;
// int accel_x;
// int accel_y;
// int accel_z;
// int magnetom_x;
// int magnetom_y;
// int magnetom_z;
// float c_magnetom_x;
// float c_magnetom_y;
// float c_magnetom_z;
// float MAG_Heading;
// 
// float Accel_Vector[3]= {0,0,0}; //Store the acceleration in a vector
// float Gyro_Vector[3]= {0,0,0};//Store the gyros turn rate in a vector
// float Omega_Vector[3]= {0,0,0}; //Corrected Gyro_Vector data
// float Omega_P[3]= {0,0,0};//Omega Proportional correction
// float Omega_I[3]= {0,0,0};//Omega Integrator
// float Omega[3]= {0,0,0};
// 
// // Euler angles
// float roll;
// float pitch;
// float yaw;
// 
// float errorRollPitch[3]= {0,0,0};
// float errorYaw[3]= {0,0,0};
// 
// unsigned int counter=0;
// uint8_t gyro_sat=0;
// 
// int SENSOR_SIGN[9] = {1,1,1,1,1,1,1,1,-1}; //Correct directions x,y,z - gyro, accelerometer, magnetometer
// 	
// float DCM_Matrix[3][3]= {
// 	{
// 	1,0,0  }
// 	,{
// 	0,1,0  }
// 	,{
// 	0,0,1  }
// };
// float Update_Matrix[3][3]={{0,1,2},{3,4,5},{6,7,8}}; //Gyros here
// 
// 
// float Temporary_Matrix[3][3]={
// 	{
// 	0,0,0  }
// 	,{
// 	0,0,0  }
// 	,{
// 	0,0,0  }
// };
// 
// 
// void Matrix_Multiply(float a[3][3], float b[3][3],float mat[3][3])
// {
// 	float op[3];
// 	for(int x=0; x<3; x++)
// 	{
// 		for(int y=0; y<3; y++)
// 		{
// 			for(int w=0; w<3; w++)
// 			{
// 				op[w]=a[x][w]*b[w][y];
// 			}
// 			mat[x][y]=0;
// 			mat[x][y]=op[0]+op[1]+op[2];
// 			
// 			float test=mat[x][y];
// 		}
// 	}
// }
// 
// /**************************************************/
// float Vector_Dot_Product(float vector1[3],float vector2[3])
// {
// 	float op=0;
// 	
// 	for(int c=0; c<3; c++)
// 	{
// 		op+=vector1[c]*vector2[c];
// 	}
// 	
// 	return op;
// }
// 
// //Computes the cross product of two vectors
// void Vector_Cross_Product(float vectorOut[3], float v1[3],float v2[3])
// {
// 	vectorOut[0]= (v1[1]*v2[2]) - (v1[2]*v2[1]);
// 	vectorOut[1]= (v1[2]*v2[0]) - (v1[0]*v2[2]);
// 	vectorOut[2]= (v1[0]*v2[1]) - (v1[1]*v2[0]);
// }
// 
// //Multiply the vector by a scalar.
// void Vector_Scale(float vectorOut[3],float vectorIn[3], float scale2)
// {
// 	for(int c=0; c<3; c++)
// 	{
// 		vectorOut[c]=vectorIn[c]*scale2;
// 	}
// }
// 
// void Vector_Add(float vectorOut[3],float vectorIn1[3], float vectorIn2[3])
// {
// 	for(int c=0; c<3; c++)
// 	{
// 		vectorOut[c]=vectorIn1[c]+vectorIn2[c];
// 	}
// }
// void Compass_Heading()
// {
// 	float MAG_X;
// 	float MAG_Y;
// 	float cos_roll;
// 	float sin_roll;
// 	float cos_pitch;
// 	float sin_pitch;
// 	
// 	cos_roll = cos(roll);
// 	sin_roll = sin(roll);
// 	cos_pitch = cos(pitch);
// 	sin_pitch = sin(pitch);
// 	
// 	// Tilt compensated Magnetic filed X:
// 	MAG_X = c_magnetom_x*cos_pitch+c_magnetom_y*sin_roll*sin_pitch+c_magnetom_z*cos_roll*sin_pitch;
// 	// Tilt compensated Magnetic filed Y:
// 	MAG_Y = c_magnetom_y*cos_roll-c_magnetom_z*sin_roll;
// 	// Magnetic Heading
// 	MAG_Heading = atan2(-MAG_Y,MAG_X);
// }
// 
// 
// void Normalize(void)
// {
//   float error=0;
//   float temporary[3][3];
//   float renorm=0;
//   
//   error= -Vector_Dot_Product(&DCM_Matrix[0][0],&DCM_Matrix[1][0])*.5; //eq.19
// 
//   Vector_Scale(&temporary[0][0], &DCM_Matrix[1][0], error); //eq.19
//   Vector_Scale(&temporary[1][0], &DCM_Matrix[0][0], error); //eq.19
//   
//   Vector_Add(&temporary[0][0], &temporary[0][0], &DCM_Matrix[0][0]);//eq.19
//   Vector_Add(&temporary[1][0], &temporary[1][0], &DCM_Matrix[1][0]);//eq.19
//   
//   Vector_Cross_Product(&temporary[2][0],&temporary[0][0],&temporary[1][0]); // c= a x b //eq.20
//   
//   renorm= .5 *(3 - Vector_Dot_Product(&temporary[0][0],&temporary[0][0])); //eq.21
//   Vector_Scale(&DCM_Matrix[0][0], &temporary[0][0], renorm);
//   
//   renorm= .5 *(3 - Vector_Dot_Product(&temporary[1][0],&temporary[1][0])); //eq.21
//   Vector_Scale(&DCM_Matrix[1][0], &temporary[1][0], renorm);
//   
//   renorm= .5 *(3 - Vector_Dot_Product(&temporary[2][0],&temporary[2][0])); //eq.21
//   Vector_Scale(&DCM_Matrix[2][0], &temporary[2][0], renorm);
// }
// 
// /**************************************************/
// void Drift_correction(void)
// {
//   float mag_heading_x;
//   float mag_heading_y;
//   float errorCourse;
//   //Compensation the Roll, Pitch and Yaw drift. 
//   static float Scaled_Omega_P[3];
//   static float Scaled_Omega_I[3];
//   float Accel_magnitude;
//   float Accel_weight;
//   
//   
//   //*****Roll and Pitch***************
// 
//   // Calculate the magnitude of the accelerometer vector
//   Accel_magnitude = sqrt(Accel_Vector[0]*Accel_Vector[0] + Accel_Vector[1]*Accel_Vector[1] + Accel_Vector[2]*Accel_Vector[2]);
//   //Accel_magnitude = Accel_magnitude / GRAVITY; // Scale to gravity.
//   // Dynamic weighting of accelerometer info (reliability filter)
//   // Weight for accelerometer info (<0.5G = 0.0, 1G = 1.0 , >1.5G = 0.0)
//   if (1 - 2*abs(1 - Accel_magnitude) > 1)
// 	Accel_weight=1;
// 	
//   if (1 - 2*abs(1 - Accel_magnitude) < 0)
// 	Accel_weight = 0;
// 
//   //Accel_weight = constrain(1 - 2*abs(1 - Accel_magnitude),0,1);  //  
// 
//   Vector_Cross_Product(&errorRollPitch[0],&Accel_Vector[0],&DCM_Matrix[2][0]); //adjust the ground of reference
//   Vector_Scale(&Omega_P[0],&errorRollPitch[0],Kp_ROLLPITCH*Accel_weight);
//   
//   Vector_Scale(&Scaled_Omega_I[0],&errorRollPitch[0],Ki_ROLLPITCH*Accel_weight);
//   Vector_Add(Omega_I,Omega_I,Scaled_Omega_I);     
//   
//   //*****YAW***************
//   // We make the gyro YAW drift correction based on compass magnetic heading
//  
//   mag_heading_x = cos(MAG_Heading);
//   mag_heading_y = sin(MAG_Heading);
//   errorCourse=(DCM_Matrix[0][0]*mag_heading_y) - (DCM_Matrix[1][0]*mag_heading_x);  //Calculating YAW error
//   Vector_Scale(errorYaw,&DCM_Matrix[2][0],errorCourse); //Applys the yaw correction to the XYZ rotation of the aircraft, depeding the position.
//   
//   Vector_Scale(&Scaled_Omega_P[0],&errorYaw[0],Kp_YAW);//.01proportional of YAW.
//   Vector_Add(Omega_P,Omega_P,Scaled_Omega_P);//Adding  Proportional.
//   
//   Vector_Scale(&Scaled_Omega_I[0],&errorYaw[0],Ki_YAW);//.00001Integrator
//   Vector_Add(Omega_I,Omega_I,Scaled_Omega_I);//adding integrator to the Omega_I
// }
// /**************************************************/
// /*
// void Accel_adjust(void)
// {
//  Accel_Vector[1] += Accel_Scale(speed_3d*Omega[2]);  // Centrifugal force on Acc_y = GPS_speed*GyroZ
//  Accel_Vector[2] -= Accel_Scale(speed_3d*Omega[1]);  // Centrifugal force on Acc_z = GPS_speed*GyroY 
// }
// */
// /**************************************************/
// 
// void Matrix_update(void)
// {
//    
//   Vector_Add(&Omega[0], &Gyro_Vector[0], &Omega_I[0]);  //adding proportional term
//   Vector_Add(&Omega_Vector[0], &Omega[0], &Omega_P[0]); //adding Integrator term
// 
//   //Accel_adjust();    //Remove centrifugal acceleration.   We are not using this function in this version - we have no speed measurement
//   
// // #if OUTPUTMODE==1         
//   Update_Matrix[0][0]=0;
//   Update_Matrix[0][1]=-G_Dt*Omega_Vector[2];//-z
//   Update_Matrix[0][2]=G_Dt*Omega_Vector[1];//y
//   Update_Matrix[1][0]=G_Dt*Omega_Vector[2];//z
//   Update_Matrix[1][1]=0;
//   Update_Matrix[1][2]=-G_Dt*Omega_Vector[0];//-x
//   Update_Matrix[2][0]=-G_Dt*Omega_Vector[1];//-y
//   Update_Matrix[2][1]=G_Dt*Omega_Vector[0];//x
//   Update_Matrix[2][2]=0;
// //  #else                    // Uncorrected data (no drift correction)
// //   Update_Matrix[0][0]=0;
// //   Update_Matrix[0][1]=-G_Dt*Gyro_Vector[2];//-z
// //   Update_Matrix[0][2]=G_Dt*Gyro_Vector[1];//y
// //   Update_Matrix[1][0]=G_Dt*Gyro_Vector[2];//z
// //   Update_Matrix[1][1]=0;
// //   Update_Matrix[1][2]=-G_Dt*Gyro_Vector[0];
// //   Update_Matrix[2][0]=-G_Dt*Gyro_Vector[1];
// //   Update_Matrix[2][1]=G_Dt*Gyro_Vector[0];
// //   Update_Matrix[2][2]=0;
// //  #endif
// 
//   Matrix_Multiply(DCM_Matrix,Update_Matrix,Temporary_Matrix); //a*b=c
// 
//   for(int x=0; x<3; x++) //Matrix Addition (update)
//   {
//     for(int y=0; y<3; y++)
//     {
//       DCM_Matrix[x][y]+=Temporary_Matrix[x][y];
//     } 
//   }
// }
// 
// void Euler_angles(void)
// {
//   pitch = -asin(DCM_Matrix[2][0]);
//   roll = atan2(DCM_Matrix[2][1],DCM_Matrix[2][2]);
//   yaw = atan2(DCM_Matrix[1][0],DCM_Matrix[0][0]);
// }