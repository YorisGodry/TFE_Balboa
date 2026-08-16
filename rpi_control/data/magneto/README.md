To compile magneto:
    gcc magneto.c -lm -o magneto.exe
The -lm argument links with the math library

"nothing" is for when the robot only has the RPi attached,
"cable" is for when the cable to the DWM is attached as well
and "dwm" is for when the DWM is attached with its cable