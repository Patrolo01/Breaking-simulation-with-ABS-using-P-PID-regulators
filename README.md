# Breaking-simulation-with-ABS-using-P-PID-regulators
This project has also web app made by my friend.

Documentation of the functions (it is also in file):
withoutABS funcion simulating vehicle braking without ABS system
recives as follow:
m - mass of vehicle
 J - interia of wheel
 r - radius of wheel
 g - gravitational acceleration
 v0 - initial velocity of vehicle
 c1,c2,c3,c4 - parameters of road
 Tp - czas próbkowania
 Ts - time of simulation
 
 With controler system there are also:
 Kp - proportional gain
 Ki - integral gain
 Kd - derivative gain
 
 FUNCTION RETURNS LIST OF ELEMENTS SUCH AS:
 [0] - velocity of vehicle over time (list)
 [1] - breaking distance over time (list)
 [2] - breaking time (float)
 [3] - wheel slip over time (list)
 [4] - wheel anugular speed over time (list)
