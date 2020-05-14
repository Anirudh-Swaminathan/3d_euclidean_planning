import numpy as np
from pyrr import aabb, line, ray
from pqdict import pqdict


class MyPlanner:
    __slots__ = ['boundary', 'blocks', 'res']

    def __init__(self, boundary, blocks, res):
        self.boundary = boundary
        self.blocks = blocks
        self.res = res

    def plan(self, start, goal):
        path = [start]
        numofdirs = 26
        [dX, dY, dZ] = np.meshgrid([-1, 0, 1], [-1, 0, 1], [-1, 0, 1])
        dR = np.vstack((dX.flatten(), dY.flatten(), dZ.flatten()))
        dR = np.delete(dR, 13, axis=1)
        dR = dR / np.sqrt(np.sum(dR ** 2, axis=0))
        dR = dR * self.res

        numConsidered = 0
        for _ in range(2000):
            mindisttogoal = 1000000
            node = None
            numConsidered += 1
            for k in range(numofdirs):

                next = path[-1] + dR[:, k]
                # Check if this direction is valid
                if (next[0] < self.boundary[0, 0] or next[0] > self.boundary[0, 3] or \
                        next[1] < self.boundary[0, 1] or next[1] > self.boundary[0, 4] or \
                        next[2] < self.boundary[0, 2] or next[2] > self.boundary[0, 5]):
                    continue

                valid = True
                for k in range(self.blocks.shape[0]):
                    if (next[0] > self.blocks[k, 0] and next[0] < self.blocks[k, 3] and \
                            next[1] > self.blocks[k, 1] and next[1] < self.blocks[k, 4] and \
                            next[2] > self.blocks[k, 2] and next[2] < self.blocks[k, 5]):
                        valid = False
                        break
                if not valid:
                    continue

                # Update next node
                disttogoal = sum((next - goal) ** 2)
                if (disttogoal < mindisttogoal):
                    mindisttogoal = disttogoal
                    node = next

            if node is None:
                break

            path.append(node)

            # Check if done
            if sum((path[-1] - goal) ** 2) <= 0.1:
                break

        return np.array(path), numConsidered


class CollisionPlanner:
    __slots__ = ['boundary', 'blocks', 'res', 'blk_list']

    def __init__(self, boundary, blocks, res):
        self.boundary = boundary
        self.blocks = blocks
        self.res = res
        # create a list of AABBs for pyrr
        self.blk_list = list()
        for k in range(blocks.shape[0]):
            mi = blocks[k, :3].reshape(1, 3)
            ma = blocks[k, 3:6].reshape(1, 3)
            ab = np.vstack((mi, ma))
            abblk = aabb.create_from_points(ab)
            # print(mi.shape, ma.shape, ab.shape, abblk.shape)
            self.blk_list.append(abblk)

    def intersect(self, start, end, k):
        fst = 0
        fet = 1
        box = self.blk_list[k]
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

    def ray_intersect_aabb(self, ray, k):
        """Calculates the intersection point of a ray and an AABB
        :param numpy.array ray1: The ray to check.
        :param numpy.array aabb: The Axis-Aligned Bounding Box to check against.
        :rtype: numpy.array
        :return: Returns a vector if an intersection occurs.
            Returns None if no intersection occurs.
        """
        """
        http://gamedev.stackexchange.com/questions/18436/most-efficient-aabb-vs-ray-collision-algorithms
        """
        # this is basically "numpy.divide( 1.0, ray[ 1 ] )"
        # except we're trying to avoid a divide by zero warning
        # so where the ray direction value is 0.0, just use infinity
        # which is what we want anyway
        direction = ray[1]
        dir_fraction = np.empty(3, dtype=ray.dtype)
        dir_fraction[direction == 0.0] = 1e8
        dir_fraction[direction != 0.0] = np.divide(1.0, direction[direction != 0.0])

        aabb = self.blk_list[k]
        t1 = (aabb[0, 0] - ray[0, 0]) * dir_fraction[0]
        t2 = (aabb[1, 0] - ray[0, 0]) * dir_fraction[0]
        t3 = (aabb[0, 1] - ray[0, 1]) * dir_fraction[1]
        t4 = (aabb[1, 1] - ray[0, 1]) * dir_fraction[1]
        t5 = (aabb[0, 2] - ray[0, 2]) * dir_fraction[2]
        t6 = (aabb[1, 2] - ray[0, 2]) * dir_fraction[2]

        tmin = max(min(t1, t2), min(t3, t4), min(t5, t6))
        tmax = min(max(t1, t2), max(t3, t4), max(t5, t6))

        # if tmax < 0, ray (line) is intersecting AABB
        # but the whole AABB is behind the ray start
        if tmax < 0:
            return None

        # if tmin > tmax, ray doesn't intersect AABB
        if tmin > tmax:
            return None

        # t is the distance from the ray point
        # to intersection

        t = min(x for x in [tmin, tmax] if x >= 0)
        point = ray[0] + (ray[1] * t)
        return point

    def plan(self, start, goal):
        path = [start]
        numofdirs = 26
        [dX, dY, dZ] = np.meshgrid([-1, 0, 1], [-1, 0, 1], [-1, 0, 1])
        dR = np.vstack((dX.flatten(), dY.flatten(), dZ.flatten()))
        dR = np.delete(dR, 13, axis=1)
        dR = dR / np.sqrt(np.sum(dR ** 2, axis=0))
        dR = dR * self.res

        numConsidered = 0
        for _ in range(2000):
            mindisttogoal = 1000000
            node = None
            numConsidered += 1
            prev = path[-1]
            for j in range(numofdirs):
                next = path[-1] + dR[:, j]
                # Check if this direction is within boundaries
                if (next[0] < self.boundary[0, 0] or next[0] > self.boundary[0, 3] or \
                        next[1] < self.boundary[0, 1] or next[1] > self.boundary[0, 4] or \
                        next[2] < self.boundary[0, 2] or next[2] > self.boundary[0, 5]):
                    continue

                valid = True
                # loop through all obstacles
                for k in range(self.blocks.shape[0]):
                    # check if next node is within any one of these blocks
                    if (next[0] > self.blocks[k, 0] and next[0] < self.blocks[k, 3] and \
                            next[1] > self.blocks[k, 1] and next[1] < self.blocks[k, 4] and \
                            next[2] > self.blocks[k, 2] and next[2] < self.blocks[k, 5]):
                        valid = False
                        break
                    # check for intersection
                    # check if ray from node to next intersects AABB
                    # se = line.create_from_points(prev, next)
                    # es = line.create_from_points(next, prev)
                    # rf = ray.create_from_line(se)
                    # rb = ray.create_from_line(es)
                    # rfi = self.ray_intersect_aabb(rf, k)
                    # rbi = self.ray_intersect_aabb(rb, k)
                    rfi = self.intersect(prev, next, k)
                    rbi = self.intersect(next, prev, k)
                    # check if collision occurred in between these 2 points
                    if rfi and rbi:
                        # if rfi is not None and rbi is not None:
                        valid = False
                        break
                if not valid:
                    continue

                # Update next node
                disttogoal = sum((next - goal) ** 2)
                if (disttogoal < mindisttogoal):
                    mindisttogoal = disttogoal
                    node = next

            if node is None:
                break

            path.append(node)

            # Check if done
            if sum((path[-1] - goal) ** 2) <= 0.1:
                break

        return np.array(path), numConsidered


def thas(t):
    return t


class AStarPlanner:
    __slots__ = ['boundary', 'blocks', 'res', 'blk_list']

    def __init__(self, boundary, blocks, res):
        self.boundary = boundary
        self.blocks = blocks
        self.res = res
        # create a list of AABBs for pyrr
        self.blk_list = list()
        for k in range(blocks.shape[0]):
            mi = blocks[k, :3].reshape(1, 3)
            ma = blocks[k, 3:6].reshape(1, 3)
            ab = np.vstack((mi, ma))
            abblk = aabb.create_from_points(ab)
            # print(mi.shape, ma.shape, ab.shape, abblk.shape)
            self.blk_list.append(abblk)

    def l2_dist(self, s, e):
        """
        return the L2 distance between 2 points
        :param s: node 1
        :param e: node 2
        :return: L2 distance between s and e
        """
        return np.sqrt(sum((s - e) ** 2))

    def l_inf_dist(self, s, e):
        """
        return the L infinity distance between 2 points
        :param s: node 1
        :param e: node 2
        :return: l_inf distance between s and e
        """
        return np.amax(s - e)

    def heuristic(self, n, goal):
        """
        return the heuristic between current node and goal
        :param n: current node
        :param goal: goal node
        :return: heuristic function
        """
        # function can either be L2 or L_inf
        return self.l2_dist(n, goal)

    def intersect(self, start, end, k):
        fst = 0
        fet = 1
        box = self.blk_list[k]
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

    def reachable_goal(self, n, g):
        # check if goal is validly reachable
        for k in range(self.blocks.shape[0]):
            rfi = self.intersect(n, g, k)
            rbi = self.intersect(g, n, k)
            # check if collision occurred in between these 2 points
            if rfi and rbi:
                return False
        return True

    def plan(self, start, goal):
        numofdirs = 26
        [dX, dY, dZ] = np.meshgrid([-1, 0, 1], [-1, 0, 1], [-1, 0, 1])
        dR = np.vstack((dX.flatten(), dY.flatten(), dZ.flatten()))
        dC = np.delete(dR, 13, axis=1)
        dR = dC / np.sqrt(np.sum(dC ** 2, axis=0))
        dR = dR * self.res
        thresh = (self.res / 2.0) * np.sqrt(3)
        INF = 9999999

        numConsidered = 0
        # OPEN is a priority queue
        pq = pqdict(key=lambda x: x['f'])
        g_tau = INF
        # g value dict
        gs = dict()
        # parent dict
        parents = dict()

        # insert start node
        # s = str((0, 0, 0))
        s = (0, 0, 0)
        hs = thas(s)
        dat = dict()
        dat['ind'] = s
        dat['coord'] = start
        dat['g'] = 0.0
        gs[hs] = 0.0
        dat['h'] = self.heuristic(start, goal)
        dat['f'] = dat['g'] + dat['h']
        pd = (None, None)
        parents[hs] = pd
        pq[hs] = dat
        print(pq, len(pq))
        fin_i = None
        fin_g = None

        # while OPEN is not empty
        while len(pq) is not 0:
            i = pq.popitem()
            # print(i)
            numConsidered += 1
            i_dat = i[1]
            i_ind = np.array(i_dat['ind'])
            # i_ind = np.array(i[0])
            i_coord = np.array(i_dat['coord'])
            # i_to_g = np.sqrt(sum((i_coord - goal) ** 2))
            i_to_g = self.l2_dist(i_coord, goal)
            if i_to_g <= thresh:
                if self.reachable_goal(i_coord, goal):
                    print("Goal reached!!")
                    fin_i = tuple(i_ind)
                    fin_g = i_coord
                    assert ((g_tau - i_dat['g'] - i_to_g) <= 1e-4)
                    break
            for j in range(numofdirs):
                # next = i_coord + dR[:, j]
                next = i_coord + dC[:, j] * self.res
                next_ind = i_ind + dC[:, j]
                # Check if this direction is within boundaries
                if (next[0] < self.boundary[0, 0] or next[0] > self.boundary[0, 3] or \
                        next[1] < self.boundary[0, 1] or next[1] > self.boundary[0, 4] or \
                        next[2] < self.boundary[0, 2] or next[2] > self.boundary[0, 5]):
                    continue

                valid = True
                # loop through all obstacles
                for k in range(self.blocks.shape[0]):
                    # check if next node is within any one of these blocks
                    if (next[0] > self.blocks[k, 0] and next[0] < self.blocks[k, 3] and \
                            next[1] > self.blocks[k, 1] and next[1] < self.blocks[k, 4] and \
                            next[2] > self.blocks[k, 2] and next[2] < self.blocks[k, 5]):
                        valid = False
                        break
                    # check for intersection
                    rfi = self.intersect(i_coord, next, k)
                    rbi = self.intersect(next, i_coord, k)
                    # check if collision occurred in between these 2 points
                    if rfi and rbi:
                        valid = False
                        break
                if not valid:
                    continue

                # cs = np.array([2.07313218, 3.56684973, 0.75795371])
                # ce = np.array([2.2028887, 0.95360107, 0.75795371])
                # the potential g value
                upg = i_dat['g'] + self.l2_dist(i_coord, next)
                # heuristic for next node
                hj = self.heuristic(next, goal)
                ni = thas(tuple(next_ind))
                # ni = tuple(next_ind)
                if ni not in gs.keys():
                    gj = INF
                else:
                    gj = gs[ni]
                # checking the OPEN condition
                if upg < gj and upg + hj < g_tau:
                    # if abs(sum(i_coord - cs)) <= 1e-4 \
                    #         and abs(sum(next - ce)) <= 1e-4:
                    #     print("MONZA - Now it does NOT detect a collision!!")
                    gs[ni] = upg
                    assert (self.l2_dist(i_coord, next) <= 1e-4 + 2.0 * thresh)
                    parents[ni] = (tuple(i_ind), tuple(i_coord))
                    next_dat = dict()
                    next_dat['coord'] = next
                    next_dat['ind'] = tuple(next_ind)
                    next_dat['g'] = upg
                    next_dat['h'] = hj
                    next_dat['f'] = upg + hj
                    pq[ni] = next_dat
                    # check if newly inserted is goal
                    # if yes, update g_tau
                    if self.l2_dist(next, goal) <= 1e-4 + thresh:
                        if self.reachable_goal(next, goal):
                            g_tau = min(g_tau, upg + hj)

        # if no path exists, i.e., goal was not reached
        if fin_i is None:
            print("Goal was NOT reachable!")
            path = [start]
            print(type(path))
            return np.array(path), numConsidered

        # if path exists recover it
        path = [goal]
        print(type(path))
        path.append(fin_g)
        cur = thas(fin_i)
        # cur = fin_i
        prev = parents[cur][1]
        while prev is not None:
            print(len(path))
            print(type(prev), prev)
            if self.l2_dist(np.array(prev), path[-1]) > 2.0*thresh + 1e-4:
                print("NOOOO!")
                print("prev: {}; cur: {}\nparents[cur]: {}; hash(par): {}".format(prev, cur, parents[cur],
                                                                                  thas(parents[cur][0])))
            assert (self.l2_dist(np.array(prev), path[-1]) <= 1e-4 + 2.0 * thresh)
            path.append(np.array(prev))
            cur = thas(parents[cur][0])
            # cur = parents[cur][0]
            prev = parents[cur][1]
        path.reverse()
        path = np.array(path)
        print(type(path[0]), type(start))
        print(path[0].shape, start.shape)
        if id(cur) is not id(thas(s)):
            print("cur: {}; thas(s): {}".format(cur, thas(s)))
            print("id(cur): {}; id(s): {}".format(id(cur), id(thas(s))))
        assert (hash(cur) == hash(thas(s)))
        assert (abs(sum(path[0] - start)) < 1e-4)
        return np.array(path), numConsidered
