import numpy as np
from pyrr import aabb, line, ray


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
