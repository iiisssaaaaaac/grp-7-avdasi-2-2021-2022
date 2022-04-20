# grp-7-avdasi-2-2021-2022
This is AVDASI GRP-7 code for wing. The wing used a DC motor to actuate a flap whose position was measured with a potentiometer. 
The aileron was actuated using a standard 9g servo.


The PID library may look unchanged, but there is a feature added:
When the error passes through 0 and becomes negative (or positive, whatever is opposite to its previous sign), the integral sum is reset. 
this avoids overshooting from the integral sum taking too long to diminish. You're welcome,
Isaac
