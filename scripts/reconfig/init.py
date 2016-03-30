
from ltl_tools.ts import MotionFts, ActionModel

# # to trun python planner_agent.py B
# specify the agent plans here
init=dict()
###################
####### workspace dimension
# 175
# 105
# 35
#     40  120  200
# x:  40, 120, 200
# y: 35, 105, 175
#####################################################

#=======================================================
#=======================================================
#=======================================================
########### B motion ##########
colormap = {'ball':'red', 'obs':'black','basket':'yellow'}
B_symbols = colormap.keys()
WIDTH = 240 # cm
HEIGHT = 210 # cm
N = 6.0
RATE = 1/N
B_init_pose = (WIDTH*RATE,HEIGHT*RATE);
B_node_dict ={
			# lower three rooms
			(WIDTH*RATE,HEIGHT*RATE): set(['r1']),
			(WIDTH*3*RATE,HEIGHT*RATE): set(['r2']),
			(WIDTH*5*RATE,HEIGHT*RATE): set(['r3']),
			# cooridor three parts
			(WIDTH*RATE,HEIGHT*3*RATE): set(['c1','ball']),
			(WIDTH*3*RATE,HEIGHT*3*RATE): set(['c2',]),
			(WIDTH*5*RATE,HEIGHT*3*RATE): set(['c3']),
			# upper three rooms
			(WIDTH*RATE,HEIGHT*5*RATE): set(['r4',]),
			(WIDTH*3*RATE,HEIGHT*5*RATE): set(['r5','basket']),
			(WIDTH*5*RATE,HEIGHT*5*RATE): set(['r6']),
			}
B_motion = MotionFts(B_node_dict, B_symbols, 'office')
B_motion.set_initial(B_init_pose)
B_edge_list = [ # 1st column
			 ((WIDTH*RATE,HEIGHT*RATE), (WIDTH*RATE,HEIGHT*3*RATE)),
			 ((WIDTH*RATE,HEIGHT*3*RATE), (WIDTH*RATE,HEIGHT*5*RATE)),
			 # 2nd row
			 ((WIDTH*RATE,HEIGHT*3*RATE), (WIDTH*3*RATE,HEIGHT*3*RATE)),
			 ((WIDTH*3*RATE,HEIGHT*3*RATE), (WIDTH*5*RATE,HEIGHT*3*RATE)),
			 # 2nd column
			 ((WIDTH*3*RATE,HEIGHT*RATE), (WIDTH*3*RATE,HEIGHT*3*RATE)),
			 ((WIDTH*3*RATE,HEIGHT*3*RATE), (WIDTH*3*RATE,HEIGHT*5*RATE)),
			 # 3rd column
			 ((WIDTH*5*RATE,HEIGHT*RATE), (WIDTH*5*RATE,HEIGHT*3*RATE)),
			 ((WIDTH*5*RATE,HEIGHT*3*RATE), (WIDTH*5*RATE,HEIGHT*5*RATE)),
			 ]
B_motion.add_un_edges(B_edge_list,unit_cost=0.1)
###############################
########### B action ##########
B_action_dict={
			 'grasp': (100, 'ball', set(['grasp'])),
			 'throw': (60, 'basket', set(['throw']))
			}
B_action = ActionModel(B_action_dict)
###############################
########### B task ############
B_task = '(<> (grasp && <> throw)) && (<>[] r1)'
#B_task = '<> grasp && (<>[] r2)'
#######################
init['B']=(B_motion, B_action, B_task)

#####################################################
#=======================================================
#=======================================================
#=======================================================
########### C motion ##########
colormap = {'ball':'red', 'obs':'black','basket':'yellow'}
C_symbols = colormap.keys()
WIDTH = 240 # cm
HEIGHT = 210 # cm
N = 6.0
RATE = 1/N
C_init_pose = (WIDTH*5*RATE,HEIGHT*1*RATE);
C_node_dict ={
			# lower three rooms
			(WIDTH*RATE,HEIGHT*RATE): set(['r1']),
			(WIDTH*3*RATE,HEIGHT*RATE): set(['r2']),
			(WIDTH*5*RATE,HEIGHT*RATE): set(['r3']),
			# cooridor three parts
			(WIDTH*RATE,HEIGHT*3*RATE): set(['c1']),
			(WIDTH*3*RATE,HEIGHT*3*RATE): set(['c2']),
			(WIDTH*5*RATE,HEIGHT*3*RATE): set(['c3','cushion']),
			# upper three rooms
			(WIDTH*RATE,HEIGHT*5*RATE): set(['r4']),
			(WIDTH*3*RATE,HEIGHT*5*RATE): set(['r5','basket']),
			(WIDTH*5*RATE,HEIGHT*5*RATE): set(['r6']),
			}
C_motion = MotionFts(C_node_dict, C_symbols, 'office')
C_motion.set_initial(C_init_pose)
edge_list = [ # 1st column
			 ((WIDTH*RATE,HEIGHT*RATE), (WIDTH*RATE,HEIGHT*3*RATE)),
			 ((WIDTH*RATE,HEIGHT*3*RATE), (WIDTH*RATE,HEIGHT*5*RATE)),
			 # 2nd row
			 ((WIDTH*RATE,HEIGHT*3*RATE), (WIDTH*3*RATE,HEIGHT*3*RATE)),
			 ((WIDTH*3*RATE,HEIGHT*3*RATE), (WIDTH*5*RATE,HEIGHT*3*RATE)),
			 # 2nd column
			 ((WIDTH*3*RATE,HEIGHT*RATE), (WIDTH*3*RATE,HEIGHT*3*RATE)),
			 ((WIDTH*3*RATE,HEIGHT*3*RATE), (WIDTH*3*RATE,HEIGHT*5*RATE)),
			 # 3rd column
			 ((WIDTH*5*RATE,HEIGHT*RATE), (WIDTH*5*RATE,HEIGHT*3*RATE)),
			 ((WIDTH*5*RATE,HEIGHT*3*RATE), (WIDTH*5*RATE,HEIGHT*5*RATE)),
			 ]
C_motion.add_un_edges(edge_list,unit_cost=0.1)
###############################
########### B action ##########
C_action_dict={
			 'grasp': (100, 'ball', set(['grasp'])),
			 'throw': (60, 'basket', set(['throw'])),
			 'crouch': (60, 'cushion', set(['crouch'])),
			}
C_action = ActionModel(C_action_dict)
###############################
########### B task ############
C_task = '(<> (crouch && <> (throw))) && (<>[] r3)'
#######################
init['C']=(C_motion, C_action, C_task)
#####################################################
