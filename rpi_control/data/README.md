These folders contain data and graphs from various experiments in rpi_control/RPi/experiments.



The "magneto" contains magneto.c that allows for calibrating the magnetometer.

This needs to be done for the magnetometer to give sensible results:
- gather data by turning the robot in all directions
- feed that data to magneto.c
- copy the matrix in the in the rpi_control/RPi/core/magnetometer.py file

