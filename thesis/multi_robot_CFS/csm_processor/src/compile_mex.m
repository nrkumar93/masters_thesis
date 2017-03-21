% mex -I/home/prasanna/catkin_ws/src/bnr_workspace/thesis/multi_robot_CFS/csm_processor/include/ ...
%     -I/home/prasanna/catkin_ws/src/bnr_workspace/thesis/multi_robot_CFS/csm/include/ ...
%     -I/opt/ros/indigo/include ...
%     -I/usr/include/eigen3 -I/usr/include ...
%     -L/home/prasanna/catkin_ws/devel/lib ...
%     -lcsm_processor.so ...
%     -l:/usr/local/lib/libgtsamDebug.so.4.0.0 ...
%     mex_csm_caller.cpp ...
%     -v

mex -I/home/prasanna/catkin_ws/src/bnr_workspace/thesis/multi_robot_CFS/csm_processor/include/ ...
    -I/home/prasanna/catkin_ws/src/bnr_workspace/thesis/multi_robot_CFS/csm/include/ ...
    -I/opt/ros/indigo/include ...
    -I/usr/include/eigen3 -I/usr/include ...
    -l:/usr/local/lib/libgtsamDebug.so.4.0.0 ...
    -lcsm_processor.so ...
    mex_csm_caller.cpp ...
    -v