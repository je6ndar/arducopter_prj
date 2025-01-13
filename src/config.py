SITL = True

HOVER = False
THROTTLE_TUNE = True

t = 0.01

#PIDs for Position Controller
pos_roll_P = 1
pos_roll_I = 0
pos_roll_D = 0

pos_pitch_P = 1
pos_pitch_I = 0
pos_pitch_D = 0

#Roll PID
roll_P = 10
roll_I = 6
roll_D = 0.001
roll_range = 0.2*(roll_P+ roll_I*t + roll_D/t)  #assumption that maximum error can be 0.2 radians  

#Pitch PID. Assume drone is symmetrical for roll and pitch 
pitch_P = roll_P
pitch_I = roll_I
pitch_D = roll_D
pitch_range = roll_range

#Yaw PID
yaw_P = 10
yaw_I = 6
yaw_D = 0.0001
yaw_range = 0.05*(yaw_P + yaw_I*t + yaw_D/t)

#Throttle PID
throttle_P = 1.8
throttle_I = 200
throttle_D = 0.00125
throttle_range = 1*(throttle_P + throttle_I*t + throttle_D/t)   #assumption that maximum ascend/descend rate is 0.5 m/t, t is defined in the beginning of the file

#zigver nicolas step response mode acro