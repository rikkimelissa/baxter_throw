#! /usr/bin/env sh

rostopic echo  /robot/limb/right/follow_joint_trajectory/feedback/feedback/actual/positions > /home/rikki/repo/baxter_ws/src/baxter_throw/pos_actual1.txt
rostopic echo  /robot/limb/right/follow_joint_trajectory/feedback/feedback/desired/positions > /home/rikki/repo/baxter_ws/src/baxter_throw/pos_desired1.txt
rostopic echo  /robot/limb/right/follow_joint_trajectory/feedback/feedback/desired/velocities > /home/rikki/repo/baxter_ws/src/baxter_throw/vel_desired1.txt

