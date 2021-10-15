class MotionPlanner:
  def __init__ (self, robot, ps):
    self.robot = robot
    self.ps = ps

  def solveBiRRT (self, maxIter = float("inf")):
    self.ps.prepareSolveStepByStep ()
    finished = False

    # In the framework of the course, we restrict ourselves to 2 connected components.
    nbCC = self.ps.numberConnectedComponents ()
    if nbCC != 2:
      raise Exception ("There should be 2 connected components.")

    iter = 0
    while True:

      #### RRT begin
      newConfigs = list ()
      q_rand = self.robot.shootRandomConfig()
      for i in range(self.ps.numberConnectedComponents()):
        q_near, _ = self.ps.getNearestConfig(q_rand,connectedComponentId=i)
        valid, path, _ = self.ps.directPath(q_near,q_rand,True)
        if not valid:
          l = self.ps.pathLength (path)
          q_new = self.ps.configAtParam (path, l)
        else:
          q_new = q_rand
        if q_new != q_near :
          self.ps.addConfigToRoadmap(q_new)
          self.ps.addEdgeToRoadmap(q_near,q_new,path,True)
          newConfigs.append(q_new)
        else:
          newConfigs.append(None)

        
      ## Try connecting the new nodes together
      for i,q in enumerate(newConfigs):
        if not q is None:
          q_near, _ = self.ps.getNearestConfig(q, connectedComponentId=1-i)
          valid, path, _ = self.ps.directPath (q, q_near, True)
          if valid:
            self.ps.addEdgeToRoadmap (q, q_near, path, True)
        
      #### RRT end

      ## Check if the problem is solved.
      nbCC = self.ps.numberConnectedComponents ()
      if nbCC == 1:
        # Problem solved
        finished = True
        break
      iter = iter + 1
      if iter > maxIter:
        break
    if finished:
        self.ps.finishSolveStepByStep ()
        return self.ps.numberPaths () - 1

  def solvePRM (self):
    self.ps.prepareSolveStepByStep ()
    #### PRM begin
    #### PRM end
    self.ps.finishSolveStepByStep ()
