# Breaking-simulation-with-ABS-using-P-PID-regulators
This project has also web app made by my friend (but unfortunately the code is on his computer).

Documentation of the functions (it is also in file):

withoutABS funcion simulating vehicle braking without ABS system recives as follow: <br>
m - mass of vehicle <br>
 J - interia of wheel <br>
 r - radius of wheel<br>
 g - gravitational acceleration<br>
 v0 - initial velocity of vehicle<br>
 c1,c2,c3,c4 - parameters of road<br>
 Tp - sampling time<br>
 Ts - time of simulation<br>
 
 With controler system there are also:<br>
 Kp - proportional gain<br>
 Ki - integral gain<br>
 Kd - derivative gain<br>
 
 FUNCTION RETURNS LIST OF ELEMENTS SUCH AS:<br>
 [0] - velocity of vehicle over time (list)<br>
 [1] - breaking distance over time (list)<br>
 [2] - breaking time (float)<br>
 [3] - wheel slip over time (list)<br>
 [4] - wheel anugular speed over time (list)<br>
