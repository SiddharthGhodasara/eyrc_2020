search_mode=OPTIMIZE_MAX_JOINT
srdf_filename=ur_robotiq.srdf
robot_name_in_srdf=ur_robotiq
moveit_config_pkg=ur_robotiq_moveit_config
robot_name=ur_robotiq
planning_group_name=arm
ikfast_plugin_pkg=ur_robotiq_arm_ikfast_plugin
base_link_name=base_link
eef_link_name=tool0
ikfast_output_path=/home/siddharth/catkin_ws/src/sahayak_bot/ebot_description/urdf/ur_robotiq_arm_ikfast_plugin/src/ur_robotiq_arm_ikfast_solver.cpp

rosrun moveit_kinematics create_ikfast_moveit_plugin.py\
  --search_mode=$search_mode\
  --srdf_filename=$srdf_filename\
  --robot_name_in_srdf=$robot_name_in_srdf\
  --moveit_config_pkg=$moveit_config_pkg\
  $robot_name\
  $planning_group_name\
  $ikfast_plugin_pkg\
  $base_link_name\
  $eef_link_name\
  $ikfast_output_path
