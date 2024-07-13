# Autonomous-vehicle
****MATLAB implementation of RE-RRT* algorithms****
**What is RE-RRT**
•	The proposed approach RE-RRT* is the extension of RRT* and focus mainly on the convergence speed and unnecessary search. We have evaluated this algorithm in terms of convergence speed and unnecessary search where the proposed approach outperforms the RRT*.
•	RE-RRT* algorithm will initially consider the whole path from the start to the goal point and will divide it into small segment. Once the path is segmented then the vehicle will analyze the first segment for an obstacle and then it will move forward if there is no obstacle within the observation segment.
**How to use**
The original package contains 4 files containing algorithm
•	RE_RRT1.m
•	RE_RRTstar.m
•	RE_RRT_star_Complex.m
•	RE_RRT_Star_Narrowpath.m
and 1 files that enable algorithm to solve problems (avoid obstacle collision) for autonomous vehicle 
•	CollisionChecking.m

**Map**
PNG and JPG files contain map images.
**Note**
Place your algorithm, corresponding map and collision checking code in same folder to run the matlab code without any problem.
