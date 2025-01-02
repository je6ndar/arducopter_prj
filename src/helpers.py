import numpy as np
#from scipy.integrate import trapezoid

#returns pid, previous error based on the given params 
def get_pid(current_error, previous_error, P, I, D, dt):
    dErr = current_error - previous_error
    p_term = P * current_error
    i_term = I * dErr * dt                      #simple trapezoid rule
    d_term = D * dErr / dt
    pid = p_term + i_term + d_term
    return pid

#k is the theoretical range, e.g. we can get pid values in the range [-k,k]
def map_pid_to_pwm_dynamic(pid_output, k):
    pwm_min = 1000
    pwm_max = 2000
    # Clamp PID output
    pid_clamped = max(-k, min(pid_output, k))
    # Map to PWM range
    pwm = pwm_min + ((pid_clamped + k) / (2 * k)) * (pwm_max - pwm_min)
    return pwm

def servo_raw_to_rc_level(servo_raw_msg):
    m = servo_raw_msg
    m.to_dict() # some bug in mavlink that servo9_raw is not visible
    #print('hasttr', hasattr(m, 'servo9_raw'))

    if not hasattr(m, 'servo11_raw'):
        return None
    if m.servo11_raw < 1250: return "EV_RC_LOW"
    if 1250 <=  m.servo11_raw <= 1750: return "EV_RC_MED"
    if m.servo11_raw > 1750: return "EV_RC_HIGH"

    return None


def euler_to_dcm(euler):
    """
    Create DCM from euler angles
    :param euler: array [roll, pitch, yaw] in rad
    :returns: 3x3 dcm array
    """
    assert(len(euler) == 3)
    phi = euler[0]
    theta = euler[1]
    psi = euler[2]
    dcm = np.zeros([3, 3])
    c_phi = np.cos(phi)
    s_phi = np.sin(phi)
    c_theta = np.cos(theta)
    s_theta = np.sin(theta)
    c_psi = np.cos(psi)
    s_psi = np.sin(psi)

    dcm[0][0] = c_theta * c_psi
    dcm[0][1] = -c_phi * s_psi + s_phi * s_theta * c_psi
    dcm[0][2] = s_phi * s_psi + c_phi * s_theta * c_psi

    dcm[1][0] = c_theta * s_psi
    dcm[1][1] = c_phi * c_psi + s_phi * s_theta * s_psi
    dcm[1][2] = -s_phi * c_psi + c_phi * s_theta * s_psi

    dcm[2][0] = -s_theta
    dcm[2][1] = s_phi * c_theta
    dcm[2][2] = c_phi * c_theta
    return dcm




def low_pass_filter(prev_measurement, current_measurement, alpha):
    estimated_measurement = alpha*prev_measurement + (1-alpha)*current_measurement
    return estimated_measurement