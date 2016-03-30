from ltl_tools.ts import MotionFts, ActionModel
#=======================================================
# workspace layout for both agents
#         naor1     naor2
#  pyr2         ||         oyr2
#           pyr3||oyr3
#               ||
#  pyr1     pyr0||oyr0     oyr1
# ======================================================
init=dict()
#=======================================================
# Model and task for Observing Youbot (OY)
#  |         oyr2
#  |oyr3
#  |
#  |oyr0     oyr1
################ OY motion  ##################
OY_node=[(-0.7,0.8,0.0),(-0.7,1.5,-1.57),(-0.2,1.5,-1.57),(-0.91,1.08,-0.79+1.57+0.3)]
OY_label={
    OY_node[0] : set(['oyr0']),
    OY_node[1]: set(['oyr1',]),
    OY_node[2]: set(['oyr2',]),
    OY_node[3]: set(['oyr3',]),
}
OY_init_pose=OY_node[0]
OY_symbols=set(['oyr0','oyr1','oyr2','oyr3'])
OY_motion=MotionFts(OY_label,OY_symbols,'OY-workspace')
OY_motion.set_initial(OY_init_pose)
#B_edge=[(B_node[0],B_node[1]),(B_node[1],B_node[2]),(B_node[1],B_node[3]),(B_node[2],B_node[3]),(B_node[4],B_node[3]),(B_node[2],B_node[4])]
OY_edge=[(OY_node[0],OY_node[1]),(OY_node[1],OY_node[2]),(OY_node[2],OY_node[3]),
(OY_node[1],OY_node[3]),]
OY_motion.add_un_edges(OY_edge,unit_cost=10.0)
########### OY action ##########
OY_action_label={
             'oyobsgrasp': (100, 'oyball', set(['oyobsgrasp'])),
            }
OY_action = ActionModel(OY_action_label)
########### OY task ############
#OY_task = '(<> grasp) && ([]<> r1) && ([]<> r2)'
OY_task = '<>(oyr1 && <> (oyr2 && <> oyobsgrasp)) && ([]<> oyr1) && ([]<> oyr2)'
########### OY initialize ############
init['OY']=(OY_motion, OY_action, OY_task)

#=======================================================
# Model and task for Pointing Youbot (PY)
#  pyr2         |
#           pyr3|
#               |
#  pyr1     pyr0|
########### PY motion  ##########
PY_node=[(-0.7,0.0,0.0),(-1.0,-1.0,0.0),(-0.0,-1.0,0.0),(-0.5,-0.8,-2.6+1.57)]
PY_label={
    PY_node[0]: set(['pyr0']),
    PY_node[1]: set(['pyr1',]),
    PY_node[2]: set(['pyr2',]),
    PY_node[3]: set(['pyr3',]), #'pyball'
}
PY_init_pose=PY_node[0]
PY_symbols=set(['pyr0','pyr1','pyr2','pyr3'])
PY_motion=MotionFts(PY_label,PY_symbols,'PY-workspace')
PY_motion.set_initial(PY_init_pose)
#B_edge=[(B_node[0],B_node[1]),(B_node[1],B_node[2]),(B_node[1],B_node[3]),(B_node[2],B_node[3]),(B_node[4],B_node[3]),(B_node[2],B_node[4])]
PY_edge=[(PY_node[0],PY_node[1]),(PY_node[1],PY_node[2]),(PY_node[2],PY_node[3]),
(PY_node[1],PY_node[3]),]
PY_motion.add_un_edges(PY_edge,unit_cost=10.0)
########### PY action ##########
PY_action_label={
             'pypoint': (100, 'pyball', set(['pypoint'])),
            }
PY_action = ActionModel(PY_action_label)
########### PY task ############
#PY_task = '(<> grasp) && ([]<> r1) && ([]<> r2)'
PY_task = '<>(pyr1 && (<> pyr2 && <> (pypoint))) && ([]<> pyr1) && ([]<> pyr2)'
########### PY initialize ############
init['PY']=(PY_motion, PY_action, PY_task)



#=======================================================
# Model and task for NAO (nao)
#      naor1    ||naor0        naor2
################ OY motion  ##################
NAO_node=[(-0.45,-1.1,0.0),(-0.0,-0.5,0.0),(-0.45,-1.1,0.0)]
NAO_label={
    NAO_node[0] : set(['naor0']),
    NAO_node[1]: set(['naor1',]),
    NAO_node[2]: set(['naor2','stopsign']),
}
NAO_init_pose=NAO_node[0]
NAO_symbols=set(['naor0','naor1','naor2','stopsign'])
NAO_motion=MotionFts(NAO_label,NAO_symbols,'NAO-workspace')
NAO_motion.set_initial(NAO_init_pose)
NAO_edge=[(NAO_node[0],NAO_node[1]),(NAO_node[1],NAO_node[2]),(NAO_node[0],NAO_node[2])]
NAO_motion.add_un_edges(NAO_edge,unit_cost=10.0)
########### OY action ##########
NAO_action_label={
             'naosit': (100, 'stopsign', set(['naosit',])),
            }
NAO_action = ActionModel(NAO_action_label)
########### OY task ############
#OY_task = '(<> grasp) && ([]<> r1) && ([]<> r2)'
#NAO_task = '<>(naor1 && (<> naor0 && <> (naor2 && <> naosit))) && ([]<> naor1) && ([]<> naor2)'
NAO_task = '<>(naor1 && (<> naor0 && <> (naor2))) && ([]<> naor1) && ([]<> naor2)'
#NAO_task = '<>(naor1 && <> (naor2 && <> naosit)) && ([]<> naor1) && ([]<> naor2)'
########### OY initialize ############
init['NAO']=(NAO_motion, NAO_action, NAO_task)



#=============================================
# python adapt_planner_agent2.py PY
# when manually send a message to PY
# rostopic pub -1 knowledge_PY ltl3_NTUA/knowledge -- 'pyball' 'pyr3'
# after PY updates plan, it moves to 'pyr3' to execute its action 'pypoint',
# PY should publish message '['oyball', 'oyr3']' to topic 'knowledge_OY'
# rostopic pub -1 knowledge_OY ltl3_NTUA/knowledge -- 'oyball' 'oyr3'
# then OY would update its plan, and moves to 'oyr3' to execute its action 'oyobsgrasp'


#=========================================
# for testing 
# rostopic pub -1 activity_done_PY ltl3_NTUA/confirmation -- '0' 'goto' '10'
# rostopic pub -1 activity_done_OY ltl3_NTUA/confirmation -- '0' 'goto' '10'
