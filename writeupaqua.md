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
	cThe controller should use both the down position and the down velocity to command thrust. Ensure that the output value is indeed thrust (the drone's mass needs to be accounted for) and that the thrust includes the non-linear effects from non-zero roll/pitch angles
	
  ##BodyRate Controller	
	
	
  ### illustrations
  
  The controller we designed  that should be a proportional controller on the body rates 
  The controller should be take into account the moments of inertia of the quad as calculating the command the moments
  
  
  ## RollPitch Controller
  
   ### illustrations
  * The controller should use the acceleration and thrust commands, in addition to the vehicle attitude to output a body rate command.
  * The controller should account for the non-linear transformation from local accelerations to body rates.
  * Note that the drone's mass should be accounted for when calculating the target angles.
  
  ## Altitude Controller
  
  ### illustrations
  
 
  * The controller should use both the down position and the down velocity to command thrust. 
  * Ensure that the output value is indeed thrust (the drone's mass needs to be accounted for) and that the thrust includes the non-         linear effects from non-zero roll/pitch angles
  
  
  
  ## LateralPositon Controller
  
  ### illustrations
  
  * The controller should use the local NE position and velocity to generate a commanded local acceleration.
  
  
  ## YawController
  
  ### illustrations
  * The controller can be a linear/proportional heading controller to yaw rate commands (non-linear transformation not required).
  
  
 ## Flight Evalution 
 * Ensure that in each scenario the drone looks stable and performs the required task. Specifically check that the student's controller is able to handle the non-linearities of scenario 4 (all three drones in the scenario should be able to perform the required task with the same control gains used).
 
 
