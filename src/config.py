SITL = True

HOVER = True
THROTTLE_TUNE = False

t = 0.01

#PIDs for Position Controller
pos_roll_P = 1
pos_roll_I = 0.02
pos_roll_D = 0.01

pos_pitch_P = pos_roll_P
pos_pitch_I = pos_roll_I
pos_pitch_D = pos_roll_D

#Roll PID
roll_P = 0.9
roll_I = 3
roll_D = 0.015
roll_range = 1#0.2*(roll_P+ roll_I*t + roll_D/t)  #assumption that maximum error can be 0.2 radians  

#Pitch PID. Assume drone is symmetrical for roll and pitch 
pitch_P = roll_P
pitch_I = roll_I
pitch_D = roll_D
pitch_range = roll_range

#Yaw PID
yaw_P = 1
yaw_I = 5
yaw_D = 0.002
yaw_range = 1#0.05*(yaw_P + yaw_I*t + yaw_D/t)

#Throttle PID
throttle_P = 0.9
throttle_I = 40
throttle_D = 0.015
throttle_range = 7 #0.1*(throttle_P + throttle_I*t + throttle_D/t)   #assumption that maximum ascend/descend rate is 0.5 m/t, t is defined in the beginning of the file

#zigver nicolas step response mode acro