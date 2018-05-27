# FCND-Controls-CPP P3 #

---
## Control inputs ##
Now we need four thrust as following

* F1:front left thrust 
* F2:front right thrust
* F3:rear left thrust
* F4:rear left thrust
---

## Controllers ###
---
### GenerateMotorCommands ###
Convert a desired 3-axis moment and collective thrust command to individual motor thrust commands
### illustraion about a quadrotor ###
* Four input forces
* Six output coodinates
* The thrust and moments should be converted to the appropriate 4 different desired thrust forces for the moments. Ensure that the dimensions of the drone are properly accounted for when calculating thrust from moments.

### The Details of method of calculatioN ###
* Get the distance of the propeller location and easy to know the distance between x-axis and propeller location is half of the distance between the agjacent propellers at 45 degrees relative to each axis
* Code 

`float l = L / sqrt(2.F);`

* caculate the Ft,Fp,Fq,Fr
* caculate the F1,F2,F3,F4
* Code & Helfful Equation

`float Ft = collThrustCmd;
`float Fp = momentCmd.x / l;
	`float Fq = momentCmd.y / l;
	`float Fr = momentCmd.z / kappa;

	float F1 = (Ft + Fp + Fq - Fr) / 4;
	float F2 = F1 - (Fp - Fr) / 2;
        float F3 = Ft - F1 - F2 - F4;
        float F4 = (Ft - Fp) / 2 - F2;

	cmd.desiredThrustsN[0] = CONSTRAIN(F1, minMotorThrust, maxMotorThrust); // front left
	cmd.desiredThrustsN[1] = CONSTRAIN(F2, minMotorThrust, maxMotorThrust); // front right
	cThe controller should use both the down position and the down velocity to command thrust. Ensure that the output value is indeed thrust (the drone's mass needs to be accounted for) and that the thrust includes the non-linear effects from non-zero roll/pitch angles
	

  
  ## RollPitch Controller
  
   ### illustrations
  * The controller should use the acceleration and thrust commands, in addition to the vehicle attitude to output a body rate command.
  * The controller should account for the non-linear transformation from local accelerations to body rates.
  * Note that the drone's mass should be accounted for when calculating the target angles.
  
   ``if ( collThrustCmd > 0 ) { 
    float c = - collThrustCmd / mass; 
    
    float b_x_cmd = CONSTRAIN(accelCmd.x / c, -maxTiltAngle, maxTiltAngle);
    
    float b_x_err = b_x_cmd - R(0,2); float b_x_p_tm = kpBank * b_x_err;
    
    float b_y_cmd = CONSTRAIN(accelCmd.y / c, -maxTiltAngle, maxTiltAngle);
    
    float b_y_err = b_y_cmd - R(1,2);
    
    
    loat b_y_p_tm = kpBank * b_y_err;`

    `pqrCmd.x = (R(1,0) * b_x_p_tm - R(0,0) * b_y_p_tm) / R(2,2);
    
    pqrCmd.y = (R(1,1) * b_x_p_tm - R(0,1) * b_y_p_tm) / R(2,2);
    } 
    else {
  
    pqrCmd.x = 0.0; pqrCmd.y = 0.0; 
    }
  
    pqrCmd.z = 0;``
  
  
  
  
       
## Altitude Controller
  
  ### illustrations
  
 
  * The controller should use both the down position and the down velocity to command thrust. 
  * Ensure that the output value is indeed thrust (the drone's mass needs to be accounted for) and that the thrust includes the non-         linear effects from non-zero roll/pitch angles
  


### Code


``float QuadControl::AltitudeControl(float posZCmd, float velZCmd, float posZ, float velZ, Quaternion<float> attitude, float accelZCmd,     float dt) 
 Mat3x3F R = attitude.RotationMatrix_IwrtB(); 
 
 float thrust = 0;
 
 float z_err = posZCmd - posZ; 
 
 float p_tm = kpPosZ * z_err;
 
 float z_dot_err = velZCmd - velZ; ;
 
 float d_tm = kpVelZ z_dot_err + velZ; 
 
 float i_tm = KiPosZ integratedAltitudeError;
 
 float b_z = R(2,2);
 
 float u_1_bar = p_tm + d_tm + i_tm + accelZCmd;
 
 float acc = ( u_1_bar - CONSintegratedAltitudeError += z_err * dtT_GRAVITY ) / b_z;
 
 thrust = - mass * CONSTRAIN(acc, - maxAscentRate / dt, maxAscentRate / dt);
 
 return thrust;``

  
  ## BodyRate Controller

  ### illustrations
  
  The controller we designed  that should be a proportional controller on the body rates 
  The controller should be take into account the moments of inertia of the quad as calculating the command the moments
  
  ### Helpful Equations
  * V3F structure is the way to record the IXX,IYY,IZZ
  * KpPQR is proportional gains on angular velocity
  * M=I*Phi_dot_dot
  
  ## Code
  
  `V3F momentCmd;
   memontCmd=V3F(Ixx,Iyy,Izz)* KpPQR*(pqrCmd-pqr)`
  
  
  
  
  ## LateralPositon Controller
  
  ### illustrations
  
   * The controller should use the local NE position and velocity to generate a commanded local acceleration.
  
   * Use PD Control and FF and constrain desired acceleration and velocity
   
     
   ` `V3F kpPos; 
      
      kpPos.x = kpPosXY; 
      
      kpPos.y = kpPosXY; 
      kpPos.z = 0.f;
      V3F kpVel; 
      kpVel.x = kpVelXY; 
      kpVel.y = kpVelXY; 
      kpVel.z = 0.f;
      V3F capVelCmd; 
      if ( velCmd.mag() > maxSpeedXY ) 
     { 
     capVelCmd = velCmd.norm() * maxSpeedXY;
     } 
      else {
       capVelCmd = velCmd;
     }

     accelCmd = kpPos ( posCmd - pos ) + kpVel ( capVelCmd - vel ) + accelCmd;
     if ( accelCmd.mag() > maxAccelXY ) 
    { 
    accelCmd = accelCmd.norm() * maxAccelXY;
     }``

	

  
  ## YawController
  
  ### illustrations
  * The controller can be a linear/proportional heading controller to yaw rate commands (non-linear transformation not required).
  ---
   * Calculate a desired yaw rate to control yaw to yawCmd
   * use fmodf to unwrap the radian angle measure floot foo to range [0.b]
   * use the yaw constant to measure the yaw rate
   
   ` float error=yawCmd-yaw;
     error=fmodf(error,2*F_PI);
     yawRateCmd=KpYaw*error;
     return yawRateCmd;`
     
   
 
