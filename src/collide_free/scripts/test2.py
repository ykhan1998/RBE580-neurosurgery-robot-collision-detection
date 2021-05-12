import numpy as np
from ambf_client import Client
_client = Client()
_client.connect()
base = _client.get_obj_handle('/ambf/env/neuro_robot/base_link')
while 1:
    base.set_joint_pos(0,0)
    base.set_joint_pos(1,0)
    base.set_joint_pos(2,0)
    base.set_joint_pos(3,0)
    base.set_joint_pos(4,0)
    base.set_joint_pos(5,0)
    base.set_joint_pos(6,0)
