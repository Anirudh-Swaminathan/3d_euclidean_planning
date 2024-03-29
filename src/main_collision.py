#!/usr/bin/python

# Created by anicodebreaker on May 10, 2020
import Planner
from pathlib import Path
import numpy as np
import time
import matplotlib.pyplot as plt

plt.ion()
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from pyrr import line, ray, geometric_tests, aabb


def tic():
    return time.time()


def toc(tstart, nm=""):
    taken = time.time() - tstart
    print('%s took: %s sec.\n' % (nm, taken))
    return taken


def load_map(fname):
    """
  Loads the bounady and blocks from map file fname.

  boundary = [['xmin', 'ymin', 'zmin', 'xmax', 'ymax', 'zmax','r','g','b']]

  blocks = [['xmin', 'ymin', 'zmin', 'xmax', 'ymax', 'zmax','r','g','b'],
            ...,
            ['xmin', 'ymin', 'zmin', 'xmax', 'ymax', 'zmax','r','g','b']]
    """
    mapdata = np.loadtxt(fname, dtype={'names': ('type', 'xmin', 'ymin', 'zmin', 'xmax', 'ymax', 'zmax', 'r', 'g', 'b'),
                                       'formats': ('S8', 'f', 'f', 'f', 'f', 'f', 'f', 'f', 'f', 'f')})
    blockIdx = mapdata['type'] == b'block'
    boundary = mapdata[~blockIdx][['xmin', 'ymin', 'zmin', 'xmax', 'ymax', 'zmax', 'r', 'g', 'b']].view('<f4').reshape(
        -1, 11)[:, 2:]
    blocks = mapdata[blockIdx][['xmin', 'ymin', 'zmin', 'xmax', 'ymax', 'zmax', 'r', 'g', 'b']].view('<f4').reshape(-1,
                                                                                                                    11)[
             :, 2:]
    return boundary, blocks


def draw_map(boundary, blocks, start, goal):
    """
  Visualization of a planning problem with environment boundary, obstacle blocks, and start and goal points
    """
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    hb = draw_block_list(ax, blocks)
    hs = ax.plot(start[0:1], start[1:2], start[2:], 'ro', markersize=7, markeredgecolor='k')
    hg = ax.plot(goal[0:1], goal[1:2], goal[2:], 'go', markersize=7, markeredgecolor='k')
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_xlim(boundary[0, 0], boundary[0, 3])
    ax.set_ylim(boundary[0, 1], boundary[0, 4])
    ax.set_zlim(boundary[0, 2], boundary[0, 5])
    return fig, ax, hb, hs, hg


def draw_block_list(ax, blocks):
    '''
  Subroutine used by draw_map() to display the environment blocks
  '''
    v = np.array([[0, 0, 0], [1, 0, 0], [1, 1, 0], [0, 1, 0], [0, 0, 1], [1, 0, 1], [1, 1, 1], [0, 1, 1]],
                 dtype='float')
    f = np.array([[0, 1, 5, 4], [1, 2, 6, 5], [2, 3, 7, 6], [3, 0, 4, 7], [0, 1, 2, 3], [4, 5, 6, 7]])
    clr = blocks[:, 6:] / 255
    n = blocks.shape[0]
    d = blocks[:, 3:6] - blocks[:, :3]
    vl = np.zeros((8 * n, 3))
    fl = np.zeros((6 * n, 4), dtype='int64')
    fcl = np.zeros((6 * n, 3))
    for k in range(n):
        vl[k * 8:(k + 1) * 8, :] = v * d[k] + blocks[k, :3]
        fl[k * 6:(k + 1) * 6, :] = f + k * 8
        fcl[k * 6:(k + 1) * 6, :] = clr[k, :]

    if type(ax) is Poly3DCollection:
        ax.set_verts(vl[fl])
    else:
        pc = Poly3DCollection(vl[fl], alpha=0.25, linewidths=1, edgecolors='k')
        pc.set_facecolor(fcl)
        h = ax.add_collection3d(pc)
        return h


def intersect(start, end, box):
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


def ray_intersect_aabb(ray, aabb):
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


def check_collision(bo, bl, pth):
    """
    A method to check for any collisions with objects in the path
    :param bo: boundary
    :param bl: blocks
    :param pth: path to check collisions for
    :return: collided boolean -> True if collision occured. False if not
    """
    node = pth[0]
    # create a list of AABBs for pyrr
    blk_list = list()
    for k in range(bl.shape[0]):
        mi = bl[k, :3].reshape(1, 3)
        ma = bl[k, 3:6].reshape(1, 3)
        ab = np.vstack((mi, ma))
        abblk = aabb.create_from_points(ab)
        # print(mi.shape, ma.shape, ab.shape, abblk.shape)
        blk_list.append(abblk)

    for i in range(1, len(pth)):
        next = pth[i]
        # Check if this direction is valid
        # Checking if the considered node is outside the bounds
        if (next[0] < bo[0, 0] or next[0] > bo[0, 3] or
                next[1] < bo[0, 1] or next[1] > bo[0, 4] or
                next[2] < bo[0, 2] or next[2] > bo[0, 5]):
            print("Collision occurred at index: {}\n Path went out of bounds!".format(i))
            return True

        # loop through all the blocks in the environment
        for k in range(bl.shape[0]):
            # check if next node is inside some block
            if (bl[k, 0] < next[0] < bl[k, 3] and
                    bl[k, 1] < next[1] < bl[k, 4] and
                    bl[k, 2] < next[2] < bl[k, 5]):
                print("Collision occurred at index {}, for block index {}.\nNew node is inside the block!".format(i, k))
                return True
            # check if ray from node to next intersects AABB
            # se = line.create_from_points(node, next)
            # es = line.create_from_points(next, node)
            # rf = ray.create_from_line(se)
            # rb = ray.create_from_line(es)
            # rfi = ray_intersect_aabb(rf, blk_list[k])
            # rbi = ray_intersect_aabb(rb, blk_list[k])
            rfi = intersect(node, next, blk_list[k])
            rbi = intersect(next, node, blk_list[k])
            if rfi and rbi:
            # if rfi is not None and rbi is not None:
                # it means the collision occurred, and it occurred in-between the points
                print("Collision occurred at index {}, for block index {}.\nCollision occurred between these 2 points!"
                      .format(i, k))
                return True
        node = next
    return False


def runtest(mapfile, start, goal, bpi, bpp, a, p, mpn, verbose=True):
    """
  This function:
   * load the provided mapfile
   * creates a motion planner
   * plans a path from start to goal
   * checks whether the path is collision free and reaches the goal
   * computes the path length as a sum of the Euclidean norm of the path segments
  """
    # Load a map and instantiate a motion planner
    boundary, blocks = load_map(mapfile)
    pth_im = "{}{}/res_{}/{}/".format(bpi, a, p, mpn)
    pth_pr = "{}{}/res_{}/{}/".format(bpp, a, p, mpn)

    # TODONE: replace this with your own planner implementation
    MP = Planner.CollisionPlanner(boundary, blocks, p)

    # Display the environment
    if verbose:
        fig, ax, hb, hs, hg = draw_map(boundary, blocks, start, goal)

        # Call the motion planner
    t0 = tic()
    path, numVisNodes = MP.plan(start, goal)
    dur = toc(t0, "{} algorithm applied on {} map with resolution {}".format(a, mpn, p))

    # Plot the path
    if verbose:
        ax.plot(path[:, 0], path[:, 1], path[:, 2], 'r-')
        # save the plot to file
        try:
            Path(pth_im).mkdir(parents=True, exist_ok=True)
            plt.savefig(pth_im + "path.png", bbox_inches='tight')
            plt.show(block=True)
        except Exception as e:
            print("Error! Could not save image! Message is {}.".format(e))

    # TODONE: You should verify whether the path actually intersects any of the obstacles in continuous space
    # TODONE: You can implement your own algorithm or use an existing library for segment and
    #       axis-aligned bounding box (AABB) intersection
    collision = check_collision(boundary, blocks, path)
    if collision:
        print("Collision Occurred!")
    else:
        print("Path has no collisions!")
    goal_reached = sum((path[-1] - goal) ** 2) <= 0.1
    success = (not collision) and goal_reached
    pathlength = np.sum(np.sqrt(np.sum(np.diff(path, axis=0) ** 2, axis=1)))
    numPthNodes = len(path)
    if verbose:
        Path(pth_pr).mkdir(parents=True, exist_ok=True)
        with open(pth_pr + "props.txt", "w") as f:
            f.write(
                "{} algorithm applied on {} map with resolution {} took {} seconds to plan.\n".format(a, mpn, p, dur))
            f.write("Successful Path?: {}\n".format(success))
            f.write("Final path has collisions?: {}\n".format(collision))
            f.write("Reached Goal?: {}\n".format(goal_reached))
            f.write("Path Length: {}\n".format(pathlength))
            f.write("Number of nodes in path: {}\n".format(numPthNodes))
            f.write("Number of nodes explored: {}\n".format(numVisNodes))
    return success, pathlength


def test_single_cube(bpi, bpa, a, p, verbose=False):
    print('Running single cube test...\n')
    start = np.array([2.3, 2.3, 1.3])
    goal = np.array([7.0, 7.0, 5.5])
    map = "single_cube"
    success, pathlength = runtest('./maps/single_cube.txt', start, goal, bpi, bpa, a, p, map, verbose)
    print('Success: %r' % success)
    print('Path length: %d' % pathlength)
    print('\n')


def test_maze(bpi, bpp, a, p, verbose=False):
    print('Running maze test...\n')
    start = np.array([0.0, 0.0, 1.0])
    goal = np.array([12.0, 12.0, 5.0])
    map = "maze"
    success, pathlength = runtest('./maps/maze.txt', start, goal, bpi, bpp, a, p, map, verbose)
    print('Success: %r' % success)
    print('Path length: %d' % pathlength)
    print('\n')


def test_window(bpi, bpp, a, p, verbose=False):
    print('Running window test...\n')
    start = np.array([0.2, -4.9, 0.2])
    goal = np.array([6.0, 18.0, 3.0])
    map = "window"
    success, pathlength = runtest('./maps/window.txt', start, goal, bpi, bpp, a, p, map, verbose)
    print('Success: %r' % success)
    print('Path length: %d' % pathlength)
    print('\n')


def test_tower(bpi, bpp, a, p, verbose=False):
    print('Running tower test...\n')
    start = np.array([2.5, 4.0, 0.5])
    goal = np.array([4.0, 2.5, 19.5])
    map = "tower"
    success, pathlength = runtest('./maps/tower.txt', start, goal, bpi, bpp, a, p, map, verbose)
    print('Success: %r' % success)
    print('Path length: %d' % pathlength)
    print('\n')


def test_flappy_bird(bpi, bpp, a, p, verbose=False):
    print('Running flappy bird test...\n')
    start = np.array([0.5, 2.5, 5.5])
    goal = np.array([19.0, 2.5, 5.5])
    map = "flappy_bird"
    success, pathlength = runtest('./maps/flappy_bird.txt', start, goal, bpi, bpp, a, p, map, verbose)
    print('Success: %r' % success)
    print('Path length: %d' % pathlength)
    print('\n')


def test_room(bpi, bpp, a, p, verbose=False):
    print('Running room test...\n')
    start = np.array([1.0, 5.0, 1.5])
    goal = np.array([9.0, 7.0, 1.5])
    map = "room"
    success, pathlength = runtest('./maps/room.txt', start, goal, bpi, bpp, a, p, map, verbose)
    print('Success: %r' % success)
    print('Path length: %d' % pathlength)
    print('\n')


def test_monza(bpi, bpp, a, p, verbose=False):
    print('Running monza test...\n')
    start = np.array([0.5, 1.0, 4.9])
    goal = np.array([3.8, 1.0, 0.1])
    map = "monza"
    success, pathlength = runtest('./maps/monza.txt', start, goal, bpi, bpp, a, p, map, verbose)
    print('Success: %r' % success)
    print('Path length: %d' % pathlength)
    print('\n')


if __name__ == "__main__":
    base_pth_img = "./path_images/"
    base_pth_prop = "./path_properties/"

    # resolution of discretization of planner
    algo = "greedy_no_collide"
    # property -> resolution for A*; sampling weight to goal for RRT*
    prop = 0.5
    test_single_cube(base_pth_img, base_pth_prop, algo, prop, True)
    test_maze(base_pth_img, base_pth_prop, algo, prop, False)
    test_flappy_bird(base_pth_img, base_pth_prop, algo, prop, False)
    test_monza(base_pth_img, base_pth_prop, algo, prop, False)
    test_window(base_pth_img, base_pth_prop, algo, prop, False)
    test_tower(base_pth_img, base_pth_prop, algo, prop, False)
    test_room(base_pth_img, base_pth_prop, algo, prop, False)
