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
* The thrustand the moments should be converted 

### The Details of method of calculatioN ###
* Get the distance of the propeller location and easy to know the distance between x-axis and propeller location is half of the distance between the agjacent propellers at 45 degrees relative to each axis
* Code 

`float l = L / sqrt(2.F);`

* caculate the Ft,Fp,Fq,Fr
* caculate the F1,F2,F3,F4
* Code

`float Ft = collThrustCmd;
	float Fp = momentCmd.x / l;
	float Fq = momentCmd.y / l;
	float Fr = momentCmd.z / kappa;

	float F1 = (Ft + Fp + Fq - Fr) / 4;
	float F2 = F1 - (Fp - Fr) / 2;
  float F3 = Ft - F1 - F2 - F4;
  float F4 = (Ft - Fp) / 2 - F2;

	cmd.desiredThrustsN[0] = CONSTRAIN(F1, minMotorThrust, maxMotorThrust); // front left
	cmd.desiredThrustsN[1] = CONSTRAIN(F2, minMotorThrust, maxMotorThrust); // front right
	cmd.desiredThrustsN[2] = CONSTRAIN(F3, minMotorThrust, maxMotorThrust); // rear left
	cmd.desiredThrustsN[3] = CONSTRAIN(F4, minMotorThrust, maxMotorThrust); // rear right 
  
  ## BodyRate Controller
  Calculate a desired 3-axis moment given a desired and current body rate
  
  * illustrations
  
  The controller we designed  that should be a proportional controller on the body rates 
  The controller should be take into account the moments of inertia of the quad as calculating the command the moments
  
  
  
      
  
 
 
