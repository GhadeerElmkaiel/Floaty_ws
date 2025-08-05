
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
    
    # flaps_range = 120
    angle_to_rad = 3.14/180
    param_val = angle_to_rad*angle

    if param_val < -3.14/3 or param_val > 3.14/3:
        print("Flap angle out of range")
        return
        
    group_name = "extCtrl"
    param_name = "m"+str(flap_id)
    return set_param_value(sfloaty, group_name, param_name, param_val)


def set_flaps_angles(angles):

    # flaps_range = 120

    group_name = "extCtrl"

    for i in range(4):
        angle_to_rad = 3.14/180
        param_val = angle_to_rad*angles[i]
        if param_val < -3.14/3 or param_val > 3.14/3:
            print("Flap {} angle out of range".format(i))
            return
        
        param_name = "m"+str(i+1)
        set_param_value(sfloaty, group_name, param_name, param_val)
    return True
        

def set_param_value(scf, groupstr, namestr, value):
    cf = scf.cf
    full_name = groupstr+ "." +namestr
    print(full_name, " ", value)
    cf.param.set_value(full_name,value)
    time.sleep(0.04)
    return 1


def set_cf_param_callback(req):
    return set_param_value(sfloaty, req.group_name, req.param_name, req.value)


if __name__ == '__main__':
    # Initialize the low-level drivers
    try:
        cflib.crtp.init_drivers()
    except:
        print("Error in radio code")
    
    assert len(sys.argv)==5 or len(sys.argv)==3
    if len(sys.argv) ==3:
        id = int(sys.argv[1])
        angle = float(sys.argv[2])
        assert (id > 0 and id < 5) and (angle >=0  and angle <= 120)
    elif len(sys.argv)==5:
        angles = [float(sys.argv[1]), float(sys.argv[2]), float(sys.argv[3]), float(sys.argv[4])]


    with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:
        sfloaty = scf
        # set_param_value(scf, "extCtrl", "startLoop", 2)
        # time.sleep(1)
        
        if len(sys.argv) ==5:
            set_flaps_angles(angles) 
        if len(sys.argv) ==3:
            set_flap_angle(id, angle)
        scf.close_link()
