from math import *

GAIN_K = 0.66
CAPTOR_DISTANCE = 100 # 10cm
WHEEL_RAY = 35 # 3,5 cm

#-------- SLAVE PARAMS
Kx = 0.0355
Ki = -0.7862
Te = 0.02
#--------

####
# Bloc 1 (cf. CR)
# pos_x : Position en x
# pos_y : Position en y
# target_x : x souhaité
# target_y : y souhaité
# theta : angle en rad
####
def position_rot_speed(pos_x, pos_y, target_x, target_y, theta):
    ux = GAIN_K * (target_x - pos_x)
    uy = GAIN_K * (target_y - pos_y)
    rinvx = CAPTOR_DISTANCE * cos(theta) + CAPTOR_DISTANCE * sin(theta)
    rinvy = -sin(theta) + cos(theta)
    
    return rinvx*ux, rinvy*uy

####
# Bloc 2
# speed : v°
# rotation_speed : theta°
####
def commande(speed, rotation_speed) :
    motor_speed_left = speed + rotation_speed * WHEEL_RAY
    motor_speed_right = speed - rotation_speed * WHEEL_RAY
    return motor_speed_left, motor_speed_right

####
# Bloc 3
# vc : Vitesse consigne
# err_int : Intégrale de l'erreur
####
def servo_system(vc, err_int):
    return -Kx * vc - Ki * err_int
    
####
#
# xi_error : Erreur
# vc : Vitesse consigne
# vs : Vitesse sortie
####
def error_calc(xi_error, vc, vs): # Vitesse consigne & Vitesse sortie
    return xi_error + (vc-vs) * Te


def main():
    initial_x = 3
    initial_y = 3
    
    target_x = 6
    target_y = 7
    
    rad = 2
    xi1_error = 0
    xi2_error = 0
    
    
    vp, thetap = position_rot_speed(initial_x, initial_y, target_x, target_y, rad)
    vm_1, vm_2 = commande(vp, thetap)
    xi1_error = error_calc(xi1_error, vm_1, )
    xi2_error = error_calc(xi2_error, vm_2, )
    
    print("v1 : ", vp, ", v2 : ", vm_2, ", u1 : ", servo_system(vm_1,xi1_error), ", u2 : ", servo_system(vm_2,xi2_error)
    
    
main()