shopt -s expand_aliases
. ~/.bashrc
rr

rosservice call /reset_positions
python base_planner.py --com 1 --task DSDA --goal '35,7'
rosservice call /reset_positions
python base_planner.py --com 1 --task CSDA --goal '35,7'
rosservice call /reset_positions
python base_planner.py --com 1 --task DSPA --goal '35,7'

# rosservice call /reset_positions
# python base_planner.py --com 1 --task DSDA --goal '48,3'
# rosservice call /reset_positions
# python base_planner.py --com 1 --task CSDA --goal '48,3'
# rosservice call /reset_positions
# python base_planner.py --com 1 --task DSPA --goal '48,3'

# rosservice call /reset_positions
# python base_planner.py --com 1 --task DSDA --goal '40,1'
# rosservice call /reset_positions
# python base_planner.py --com 1 --task CSDA --goal '40,1'
# rosservice call /reset_positions
# python base_planner.py --com 1 --task DSPA --goal '40,1'

# rosservice call /reset_positions
# python base_planner.py --com 1 --task DSDA --goal '36,10'
# rosservice call /reset_positions
# python base_planner.py --com 1 --task CSDA --goal '36,10'
# rosservice call /reset_positions
# python base_planner.py --com 1 --task DSPA --goal '36,10'

# rosservice call /reset_positions
# python base_planner.py --com 1 --task DSDA --goal '30,16'
# rosservice call /reset_positions
# python base_planner.py --com 1 --task CSDA --goal '30,16'
# rosservice call /reset_positions
# python base_planner.py --com 1 --task DSPA --goal '30,16'

# rosservice call /reset_positions
# python base_planner.py --com 1 --task DSDA --goal '20,1'
# rosservice call /reset_positions
# python base_planner.py --com 1 --task CSDA --goal '20,1'
# rosservice call /reset_positions
# python base_planner.py --com 1 --task DSPA --goal '20,1'

# rosservice call /reset_positions
# python base_planner.py --com 1 --task DSDA --goal '48,18'
# rosservice call /reset_positions
# python base_planner.py --com 1 --task CSDA --goal '48,18'
# rosservice call /reset_positions
# python base_planner.py --com 1 --task DSPA --goal '48,18'

# rosservice call /reset_positions
# python base_planner.py --com 1 --task DSDA --goal '1,1'
# rosservice call /reset_positions
# python base_planner.py --com 1 --task CSDA --goal '1,1'
# rosservice call /reset_positions
# python base_planner.py --com 1 --task DSPA --goal '1,1'

# rosservice call /reset_positions
# python base_planner.py --com 1 --task DSDA --goal '13,18'
# rosservice call /reset_positions
# python base_planner.py --com 1 --task CSDA --goal '13,18'
# rosservice call /reset_positions
# python base_planner.py --com 1 --task DSPA --goal '13,18'