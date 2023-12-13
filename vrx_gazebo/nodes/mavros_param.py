#!/usr/bin/env python3
import rospy
from mavros_msgs.srv import ParamGet


class MAVROSParam:
    def __init__(self):
        self.param_get_srv = rospy.ServiceProxy('/mavros/param/get', ParamGet)
        
    def get_param(self, param_id):
        param = self.param_get_srv(param_id)
        if param.success:
            if param.value.integer != 0:
                return param.value.integer
            elif param.value.real != 0.0:
                return param.value.real
            else:
                return 0
        else:
            raise Exception("Failed to get param: {}".format(param_id))
    def print_param(self, param_id):
        # print(f"{param_id}: {self.get_param(param_id)}")
        print(f"param set-default {param_id} {self.get_param(param_id)}")
        
if __name__ == '__main__':
    rospy.init_node('mavros_param')
    mavros_param = MAVROSParam()
    mavros_param.print_param("MC_ROLLRATE_K")
    mavros_param.print_param("MC_ROLLRATE_I")
    mavros_param.print_param("MC_ROLLRATE_D")
    mavros_param.print_param("MC_PITCHRATE_K")
    mavros_param.print_param("MC_PITCHRATE_I")
    mavros_param.print_param("MC_PITCHRATE_D")
    mavros_param.print_param("MC_YAWRATE_K")
    mavros_param.print_param("MC_YAWRATE_I")
    mavros_param.print_param("MC_YAWRATE_D")
    mavros_param.print_param("MC_ROLL_P")
    mavros_param.print_param("MC_PITCH_P")
    mavros_param.print_param("MC_YAW_P")
    mavros_param.print_param("MPC_XY_VEL_P_ACC")
    mavros_param.print_param("MPC_XY_VEL_I_ACC")
    mavros_param.print_param("MPC_XY_VEL_D_ACC")
    mavros_param.print_param("MPC_Z_VEL_P_ACC")
    mavros_param.print_param("MPC_Z_VEL_I_ACC")
    mavros_param.print_param("MPC_Z_VEL_D_ACC")
    mavros_param.print_param("MPC_XY_P")
    mavros_param.print_param("MPC_Z_P")
