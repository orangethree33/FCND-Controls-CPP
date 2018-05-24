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
### specifications about a quadrotor ###
* Four input forces
* Six output coodinates
* The thrustand the moments should be converted 

### The Details of method of calculatioN ###
* Get the distance of the propeller location and easy to know the distance between x-axis and propeller location is half of the distance between the agjacent propellers at 45 degrees relative to each axis
* caculate the Ft,Fp,Fq,Fr

