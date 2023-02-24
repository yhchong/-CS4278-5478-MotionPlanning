shopt -s expand_aliases
. ~/.bashrc
rr
rosservice call /reset_positions
python base_planner.py --com 0 --task CSDA --map map3 --goal '6,1'
rosservice call /reset_positions
python base_planner.py --com 0 --task CSDA --map map3 --goal '5,7'
rosservice call /reset_positions
python base_planner.py --com 0 --task CSDA --map map3 --goal '3,7'
rosservice call /reset_positions
python base_planner.py --com 0 --task CSDA --map map3 --goal '1,9'
rosservice call /reset_positions
python base_planner.py --com 0 --task CSDA --map map3 --goal '4,5'
rosservice call /reset_positions
python base_planner.py --com 0 --task CSDA --map map3 --goal '1,1'