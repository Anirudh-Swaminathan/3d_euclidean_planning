#!/usr/bin/python

# Created by anicodebreaker on May 14, 2020
import numpy as np
from pyrr import aabb
from ompl import util as ou
from ompl import base as ob
from ompl import geometric as og

ou.setLogLevel(ou.LOG_DEBUG)


class RRTStarPlanner:
    __slots__ = ['boundary', 'blocks', 'gbias', 'blk_list', 'rtime']

    class AniMotionCheck(ob.MotionValidator):
        def __init__(self, outer_instance, p_object, *args, **kwargs):
            super().__init__(p_object, *args, **kwargs)
            self.outer_instance = outer_instance

        def checkMotion(self, s1, s2):
            a = self.outer_instance.valid_states(s1)
            b = self.outer_instance.valid_states(s2)
            if not a or not b:
                return False
            return self.outer_instance.valid_motion(s1, s2)

    def __init__(self, boundary, blocks, res):
        self.boundary = boundary
        self.blocks = blocks
        self.gbias = res
        self.rtime = 30
        # create a list of AABBs for pyrr
        self.blk_list = list()
        for k in range(blocks.shape[0]):
            mi = blocks[k, :3].reshape(1, 3)
            ma = blocks[k, 3:6].reshape(1, 3)
            ab = np.vstack((mi, ma))
            abblk = aabb.create_from_points(ab)
            # print(mi.shape, ma.shape, ab.shape, abblk.shape)
            self.blk_list.append(abblk)

    def valid_states(self, state):
        # return True
        # Check if this direction is within boundaries
        if (state[0] < self.boundary[0, 0] or state[0] > self.boundary[0, 3] or
                state[1] < self.boundary[0, 1] or state[1] > self.boundary[0, 4] or
                state[2] < self.boundary[0, 2] or state[2] > self.boundary[0, 5]):
            return False
        # return True
        # for each block, check if it is inside
        for k in range(self.blocks.shape[0]):
            # check if next node is within any one of these blocks
            if (self.blocks[k, 0] < state[0] < self.blocks[k, 3] and
                    self.blocks[k, 1] < state[1] < self.blocks[k, 4] and
                    self.blocks[k, 2] < state[2] < self.blocks[k, 5]):
                return False
        return True

    def intersect(self, start, end, box):
        fst = 0
        fet = 1
        # print(box.shape)
        bomin = aabb.minimum(box)
        bomax = aabb.maximum(box)
        # print(bomin.shape, bomax.shape)
        for i in range(3):
            bmin = bomin[i]
            bmax = bomax[i]
            si = start[i]
            ei = end[i]
            if si < ei:
                if si > bmax or ei < bmin:
                    return False
                di = ei - si
                st = (bmin - si) / di if si < bmin else 0
                et = (bmax - si) / di if ei > bmax else 1
            else:
                if ei > bmax or si < bmin:
                    return False
                di = ei - si
                st = (bmax - si) / di if si > bmax else 0
                et = (bmin - si) / di if ei < bmin else 1
            if st > fst:
                fst = st
            if et < fet:
                fet = et
            if fet < fst:
                return False
        return True

    def valid_motion(self, start, end):
        for k in range(self.blocks.shape[0]):
            # check if ray from node to next intersects AABB
            rfi = self.intersect(start, end, self.blk_list[k])
            rbi = self.intersect(end, start, self.blk_list[k])
            if rfi and rbi:
                return False
        return True

    def plan(self, start, goal):
        pathLength = 0
        # robot state space
        space = ob.RealVectorStateSpace(3)
        # bounds
        bounds = ob.RealVectorBounds(3)
        # for each dimension of the space, set the bounds according to the boundary
        for i in range(3):
            bounds.setLow(i, float(self.boundary[0, i]))
            bounds.setHigh(i, float(self.boundary[0, i+3]))
        space.setBounds(bounds)

        # space information
        si = ob.SpaceInformation(space)
        ss = og.SimpleSetup(si)

        # validity checker for collisions
        si.setStateValidityChecker(ob.StateValidityCheckerFn(self.valid_states))
        # validity checker for motion
        mot = RRTStarPlanner.AniMotionCheck(self, si)
        si.setMotionValidator(mot)
        si.setup()

        # setup start and end nodes
        startPos = ob.State(space)
        goalPos = ob.State(space)
        for i in range(3):
            startPos[i] = start[i]
            goalPos[i] = goal[i]

        # problem definition
        pdef = ob.ProblemDefinition(si)

        # setting start and goal states
        pdef.setStartAndGoalStates(startPos, goalPos)

        # optimization objective -> pathLength
        pdef.setOptimizationObjective(ob.PathLengthOptimizationObjective(si))

        # RRT* planner
        # OPTIONS - RRT, RRTConnect
        optimizingPlanner = og.RRTstar(si)

        # set instance
        optimizingPlanner.setProblemDefinition(pdef)
        optimizingPlanner.setup()

        # solve it, atleast attempt to solve it
        solved = optimizingPlanner.solve(self.rtime)
        hes = pdef.hasExactSolution()
        inde = 0
        while not hes and inde < 4:
            print("Not solved yet!")
            solved = optimizingPlanner.solve(self.rtime)
            hes = pdef.hasExactSolution()
            inde += 1

        # if no path exists, i.e., goal was not reached
        if not solved:
            print("Goal was NOT reachable!")
            path = [start]
            print(type(path))
            return np.array(path), pathLength

        print("Solution found!")
        print("Exact Solution?: {}".format(hes))
        # simplify solution
        ss.simplifySolution()
        # if path exists recover it
        path = pdef.getSolutionPath()
        if ss.haveSolutionPath():
            p2 = ss.getSolutionPath()
            print(path.printAsMatrix() == p2.printAsMatrix())
        pathLength = pdef.getSolutionPath().cost(pdef.getOptimizationObjective()).value()
        return path, pathLength
