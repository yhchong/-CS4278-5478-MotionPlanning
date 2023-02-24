shopt -s expand_aliases
. ~/.bashrc
rr

rosservice call /reset_positions
python base_planner.py --com 0 --task DSDA --map map4 --goal '1,1'
rosservice call /reset_positions
python base_planner.py --com 0 --task DSDA --map map4 --goal '9,9'
rosservice call /reset_positions
python base_planner.py --com 0 --task DSDA --map map4 --goal '9,1'
rosservice call /reset_positions
python base_planner.py --com 0 --task DSDA --map map4 --goal '7,1'
rosservice call /reset_positions
python base_planner.py --com 0 --task DSDA --map map4 --goal '5,5'
rosservice call /reset_positions
python base_planner.py --com 0 --task DSDA --map map4 --goal '5,2'

rosservice call /reset_positions
python base_planner.py --com 0 --task CSDA --map map4 --goal '1,1'
rosservice call /reset_positions
python base_planner.py --com 0 --task CSDA --map map4 --goal '9,9'
rosservice call /reset_positions
python base_planner.py --com 0 --task CSDA --map map4 --goal '9,1'
rosservice call /reset_positions
python base_planner.py --com 0 --task CSDA --map map4 --goal '7,1'
rosservice call /reset_positions
python base_planner.py --com 0 --task CSDA --map map4 --goal '5,5'
rosservice call /reset_positions
python base_planner.py --com 0 --task CSDA --map map4 --goal '5,2'

rosservice call /reset_positions
python base_planner.py --com 0 --task DSPA --map map4 --goal '1,1'
rosservice call /reset_positions
python base_planner.py --com 0 --task DSPA --map map4 --goal '9,9'
rosservice call /reset_positions
python base_planner.py --com 0 --task DSPA --map map4 --goal '9,1'
rosservice call /reset_positions
python base_planner.py --com 0 --task DSPA --map map4 --goal '7,1'
rosservice call /reset_positions
python base_planner.py --com 0 --task DSPA --map map4 --goal '5,5'
rosservice call /reset_positions
python base_planner.py --com 0 --task DSPA --map map4 --goal '5,2'