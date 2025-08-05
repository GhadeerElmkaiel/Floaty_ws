import logging
import sys
import time

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.utils import uri_helper

from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncLogger import SyncLogger

from floaty_msgs.srv import set_cf_param_srv, set_flap_angle_srv, set_flaps_angles_srv

uri = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E7E7')
# Only output errors from the logging framework
logging.basicConfig(level=logging.ERROR)
sfloaty = None


def set_flap_angle(flap_id, angle):

    if flap_id<1 or flap_id>4:
        print("Motor id not correct")
        return
    
    flaps_range = 120
    angle_to_param = 65535/flaps_range
    param_val = int(angle_to_param*angle)

    if param_val < 1 or param_val > 65534:
        print("Flap angle out of range")
        return
        
    group_name = "extModeControl"
    param_name = "motVal"+str(flap_id)
    return set_param_value(sfloaty, group_name, param_name, param_val)


def set_flaps_angles(angles):

    flaps_range = 120

    group_name = "extModeControl"

    for i in range(4):
        angle_to_param = 65535/flaps_range
        param_val = int(angle_to_param*angles[i])
        if param_val < 1 or param_val > 65534:
            print("Flap {id} angle out of range".format(i))
            return
        
        param_name = "motVal"+str(i)
        set_param_value(sfloaty, group_name, param_name, param_val)
    return True
        

def set_param_value(scf, groupstr, namestr, value):
    cf = scf.cf
    full_name = groupstr+ "." +namestr
    print(full_name, " ", value)
    cf.param.set_value(full_name,value)
    time.sleep(0.1)
    return 1


def set_cf_param_callback(req):
    return set_param_value(sfloaty, req.group_name, req.param_name, req.value)


if __name__ == '__main__':
    # Initialize the low-level drivers
    try:
        cflib.crtp.init_drivers()
    except:
        print("Error in radio code")
    
    arg1 = sys.argv[1]
    arg2 = sys.argv[2]


    with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:
        sfloaty = scf
        set_param_value(scf, "extModeControl", "startLoop", 2)
        # time.sleep(1)
        id = int(arg1)
        angle = float(arg2)
        if id == 5:
            set_flaps_angles(angle) 
        elif id < 5:
            set_flap_angle(id, angle)
