# 3d_euclidean_planning
# ECE276B Project 2 - Spring 2020 UCSD MS ECE
## Anirudh Swaminathan - aswamina@eng.ucsd.edu
This is my repository to work on the codes for ECE 276B project 2 - Motion Planning

The following are the list of source files relevant to the proper implementation of the project.

### Main Source Files

These are the main, final files for the project.

- src/main.py           -> The main.py file which contains the final implementation of my planning algorithm in the given 3D continuous space. This is a copy of main_astar.py
- src/main_astar.py     -> The main file that uses the A\* class file to run the implementation of the A\* algorithm on the given environments
- src/main_original.py  -> A copy of the original file provided to us, in case I make any mistakes
- src/Planner.py        -> Provided file that houses all the classes of all the different planners. This includes A\*, sample greedy planner and sample greedy planer with collision checking.
- src/main_ompl.py      -> The main file that uses the OMPLPlanner.py to plan the RRT\* algorithm using the OMPL library.
- src/OMPLPlanner.py    -> The file that houses the RRTStarPlanner class file to implement OMPL library code for planning.

### Auxillary Source Files

These are the auxillary source files that I implemented to build my final solution.

- src/main_collision.py   -> This file uses collision checking on the given greedy algorithm file
- src/main_greedy.py      -> This file just implements the provided greedy algorithm as a very raw baseline.

### Extra Source Files

These are the source files that I implemented to learn and understand the tools that I use for this project

 - src/ompl_demo.py  -> OMPL RRT\* algorithm demonstration from the original library

### Input Folders
 - src/maps -> These contains the different 3D environments with their AABB obstacles that is used for the project

### Output Folders
 - src/path_images     -> This folder contains the image of all the environments along with the computed path from the start to the end node. Each sub-directory contains the image for that specific algorithm. Sub-directories are a_star/ greedy/ and rrt_star/
 - src/path_properties -> This folder contains the properties, such as shortest path, path length and total number of considered nodes for each of the 7 environments. Each sub-directory contains the details for that specific algorithm. Sub-directories are a_star/ greedy/ and rrt_star/

### Instructions for running the algorithm
 - Install all the dependencies by running
 ```bash
 pip install -r requirements.txt
 ```
 - Install OMPL library with Python Bindings
   Detailed instrcutions are provided [here](https://ompl.kavrakilab.org/installation.html)

 - Run the main program using the following command
 ```bash
 python main.py
 ```

### Acknowledgements
The collision checking code in the pyrr library was buggy, and thanks to Piazza posts in the class, I was able to see a C++ implementation provided [here](http://www.garagegames.com/community/blogs/view/309), and was able to adapt it to Python for the collision checking.
