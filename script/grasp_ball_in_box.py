from math import sqrt
from hpp import Transform
from hpp.corbaserver.manipulation import ConstraintGraph, Constraints
from hpp.corbaserver import Client
Client ().problem.resetProblem ()
from manipulation import robot, vf, ps, Ground, Box, Pokeball, PathPlayer, gripperName, ballName
import sys

vf.loadEnvironmentModel (Ground, 'ground')
vf.loadEnvironmentModel (Box, 'box')
vf.moveObstacle ('box/base_link_0', [0.3+0.04, 0, 0.04, 0, 0, 0, 1])
vf.moveObstacle ('box/base_link_1', [0.3-0.04, 0, 0.04, 0, 0, 0, 1])
vf.moveObstacle ('box/base_link_2', [0.3, 0.04, 0.04, 0, 0, 0, 1])
vf.moveObstacle ('box/base_link_3', [0.3, -0.04, 0.04, 0, 0, 0, 1])

vf.loadObjectModel (Pokeball, 'pokeball')
robot.setJointBounds ('pokeball/root_joint', [-.4,.4,-.4,.4,-.1,1.,
                                              -1.0001, 1.0001,-1.0001, 1.0001,
                                              -1.0001, 1.0001,-1.0001, 1.0001,])

q1 = [0, -1.57, 1.57, 0, 0, 0, .3, 0, 0.025, 0, 0, 0, 1]

## Create graph
graph = ConstraintGraph (robot, 'graph')

#### Exercise 2

## Create constraint of relative position of the ball in the gripper when ball
## is grasped
ballInGripper = [0, .137, 0, 0.5, 0.5, -0.5, 0.5]

## Create nodes and edges
#NODES
graph.createNode (['ball-above-ground', 'grasp-placement', 'grasp', 'gripper-above-ball', 'placement'])
#EDGES
graph.createEdge ('placement', 'placement', 'transit', 1, 'placement')
graph.createEdge ('placement', 'gripper-above-ball', 'approach-ball', 1, 'placement')
graph.createEdge('gripper-above-ball', 'placement', 'move-gripper-away', 1, 'placement')
graph.createEdge('grasp-placement', 'gripper-above-ball', 'move-gripper-up', 1, 'placement')
graph.createEdge('gripper-above-ball', 'grasp-placement', 'grasp-ball', 1, 'placement')
graph.createEdge('grasp-placement', 'ball-above-ground', 'take-ball-up', 1, 'grasp')
graph.createEdge('ball-above-ground', 'grasp-placement', 'put-ball-down', 1, 'grasp')
graph.createEdge('ball-above-ground', 'grasp', 'take-ball-away', 1, 'grasp')
graph.createEdge('grasp', 'ball-above-ground', 'approach-ground', 1, 'grasp')
graph.createEdge('grasp', 'grasp', 'transfer', 1, 'grasp')

#PLACEMENT
## Create transformation constraint : ball is in horizontal plane with free
## rotation around z
ps.createTransformationConstraint ('placement', '', ballName, [0,0,0.025,0, 0, 0, 1], [False, False, True, True, True, False,])
#  Create complement constraint
ps.createTransformationConstraint ('placement/complement', '', ballName, [0,0,0.025,0, 0, 0, 1], [True, True, False, False, False, True,])
#GRIPPER ABOVE BALL                                   
# Create constraint when gripper is above the ball 
ps.createTransformationConstraint ('gripper-above-ball', gripperName, ballName, [0, .237, 0, 0.5, 0.5, -0.5, 0.5], [True, True, True, True, True, True,])                               
#BALL ABOVE GROUND
# Create constraint when gripper is in ball above ground
ps.createTransformationConstraint ('ball-above-ground', '', ballName, [0, 0, 0.125, 0, 0, 0, 1], [False, False, True, True, True, False,])                               
#GRASP
# Create constraint when gripper is in grasp
ps.createTransformationConstraint ('grasp', gripperName, ballName, ballInGripper, [True, True, True, True, True, True,])                               
                                   
ps.setConstantRightHandSide ('placement', True)
ps.setConstantRightHandSide ('placement/complement', False)
ps.setConstantRightHandSide('ball-above-ground',True)
ps.setConstantRightHandSide('gripper-above-ball',True)

## Set constraints of nodes and edges
#CONSTRAINTS ON NODES
graph.addConstraints (node='placement', constraints = Constraints (numConstraints = ['placement'],))
graph.addConstraints (node='gripper-above-ball', constraints = Constraints (numConstraints = ['gripper-above-ball','placement'],))
graph.addConstraints (node='grasp-placement', constraints = Constraints (numConstraints = ['grasp','placement'],))
graph.addConstraints (node='ball-above-ground', constraints = Constraints (numConstraints = ['ball-above-ground','grasp'],))
graph.addConstraints (node='grasp', constraints = Constraints (numConstraints = ['grasp'],))

#CONSTRAINTS ON EDGES
graph.addConstraints (edge='transit', constraints = Constraints (numConstraints = ['placement/complement']))
graph.addConstraints (edge='approach-ball', constraints = Constraints (numConstraints = ['placement/complement']))
graph.addConstraints (edge='move-gripper-away', constraints = Constraints (numConstraints = ['placement/complement']))
graph.addConstraints (edge='grasp-ball', constraints = Constraints (numConstraints = ['placement/complement']))
graph.addConstraints (edge='move-gripper-up', constraints = Constraints (numConstraints = ['placement/complement']))
graph.addConstraints (edge='take-ball-up', constraints = Constraints (numConstraints = ['placement/complement']))
graph.addConstraints (edge='put-ball-down', constraints = Constraints (numConstraints = ['placement/complement']))

#### END Exercise

ps.selectPathValidation ("Discretized", 0.01)
ps.selectPathProjector ("Progressive", 0.1)
graph.initialize ()

res, q_init, error = graph.applyNodeConstraints ('placement', q1)
q2 = q1 [::]
q2 [7] = .2

res, q_goal, error = graph.applyNodeConstraints ('placement', q2)

ps.setInitialConfig (q_init)
ps.addGoalConfig (q_goal)

# MAKES THE SOLVE AUTOMATIC
v = vf.createViewer ()
print("Trying ps.solve()...")
resultat = ps.solve()
print("ps.solve() done. Resultat : " + str(resultat))
print("Displaying path found.")
pp = PathPlayer (v)
pp(0)


  