# Autonomous Unmannned Aerial Vehicle
## StudyPlan
* Learning and implementation of planning algos (A*, RRT) to autonomously navigate between multiple waypoints.
* Implementing on 2D.
* Extending the implementation on a drone in a dense urban environment by planning a collision free path between buildings using a simulator
* Estimating position and altitude from IMU and GPS data using Extended Kalman Filter and applying controls
* Implementing on 3D.



### [Autonomous Navigation, by MATLAB](https://www.youtube.com/watch?v=QR3U1dgc5RE)

* [Part 1: What Is Autonomous Navigation?](https://youtu.be/Fw8JQ5Q-ZwU)
    1. Autonomous Navigation is the ability for a vehicle to determine its location and plan a path towards some goal without human interference.
    2. Autonomy :- The ability to make decisions and act on its own. To have a fully autonomous vehicle we have two approaches 
        1. Heuristic Approach
            * Practical Rules or behavior
            * Not guaranteed to be Optimal
            * Doesn't Need complete environment information
        2. Optimal Approach
            * Requires more environmental information 
            * Planning is achieved through optimization (Maximisation/Minimisation of an Objective Function)
    3. Heuristic Approach
    Suppose we have a maze solving vehicle whose algorithm is to keep to the left untill it reaches the goal. Except for some special cases it will reach the goal but maybe it won't follow the optimal path, even though it is solving the loop.
    ![](https://i.imgur.com/jiSjiHA.png)

    4. Optimal Approach
    In these systems the Vehicle builds/updates the model of the environment then it figues out an optimal path to reach the goal
    5. Difficulties in Autonomy
    The main difficulty is that the vehicle has to operate in an environment that is not perfectly known. So in order to create a plan it has to build up a model of the environment that is constantly changing so the nodel has to be constantly updated.
    6. Capabilities of Autonomous Systems
    Autonomous systems need to interact with the world and collect data using the sensors and then this data is interpreted into useful form (like understanding where other object are and building a map of the environment). With this the vehicle plans a path from start to goal avoiding obstacles. And then the last step is to control the vehicle to follow that plan. The actuaters impact the physical wold and the whole loop continues. 
    ![](https://i.imgur.com/SQymSV4.png)

    > NOTE : Its not usually the case that the solution is pure Heuristic or Optimal so we employ both approaches to achieve a larger goal.  
    > For eg. With an autonomous car, when it approaches a slower car it has to make a decision to whether to overtake it or to slow down. For an Optimal Solution we need knowledge beyond the front car. However it may not be possible so we follow the Hueristic Approach that if it is safe then only overtake otherwise slow down, adn once that decision has been taken then the Optimal path to the adjacent lane can be created.


* [Part 2: Understanding The Particle Filter](https://www.youtube.com/watch?v=NrzmH_yerBU)  

    1. A turtlebot is given the map of a floor but it doesn't know its position initialy.  
    2. It will use its sensors and motion models to estimate its position and orientation.   
    3. It has weighted deadreckon in its position using odometry deadreckoning. Deadreconking is when a future position is calculated using the past position and relative measurements like linear and angular velocity(However for larger timeframes error accumulates and needs to be corrected by measurement).   
    4. So Noisy Lidar + Noisy Odometry = Kalman Filter. One problem with Kalman Filter is that it expects the probability distribution to be Gaussian. So our random Dead Reckoning noise and the Lidar measurement needs to be Gaussian.Also the probability distribution of the estimated state of the robot must also be Gaussian, but that is not the case with our problem statement.
    6. We use the particle filter to handle the non gaussian probability distribution 
    
    ![](https://i.imgur.com/UFZGYrB.png)
    >For getting an actual experience on working with the Adaptive Monte Carlo Localization go to the Turtlebot localization of MATLAB [here](https://in.mathworks.com/help/nav/ug/localize-turtlebot-using-monte-carlo-localization.html#:~:text=Monte%20Carlo%20Localization%20(MCL)%20is,and%20sensing%20of%20the%20robot.)


* [Part 3: Understanding SLAM using Pose Graph Optimisation](https://www.youtube.com/watch?v=saVZtgPyyJQ) ~~Optional~~


* [Part 4: Path Planning with A* and RRT](https://www.youtube.com/watch?v=QR3U1dgc5RE&t=2s)
    1. Explore using a map for motion planning, and two approches for creating this graph
        * Search Based A*
        * Sampling based RRT and RRT*
    2. Base requirements
        * Find a path from the starting pose to the goal pose
        * For a Robot on ground 3 states (x, y, orientation)
    3. Graph based Algorithms
        * Break up the environment into discrete points/nodes,, and then finding the shortest distance to the goal considering only these nodes. We randomly choose any node move towards goal. Again a random path is choosen and if it intersects both are compared and better one is kept. 
    4. A*  
    In addition to the cost of the node and the sum of these two numbers is the absolute minimum cost of the path if there was a straight line shot to the goal.
![](https://i.imgur.com/7tCLp92.png)


    ![A* Algorithm](https://i.imgur.com/cU7yMoc.jpg)


>NOTE : A* isn't computationally feasible for complex paths. Hence sampling based algo is used.

**Rapildly Exploring Random Trees**
Randomly Sampling==>Randomly Seclecting Nodes
1. Select a random node specifying max distance.
2. Find its nearest node.
3. If there is no obstacle in the way we connect to nearest node or at max connection distance.
In RRT path wouldn't be shortest.

In **RRT*** the difference comes in connecting the nodes. We define a search radius and find can we reconnect the local nodes within the radius so as to get shortest path keeping the same tree connection. If we keep on letting the nodes branch we will eventually get an optimal path.



---
### Additional Resources

* [RRT, RRT* & Random Trees by Aaron Becker ](https://youtu.be/Ob3BIJkQJEw)

An **RT** selects a node at random from the tree and adds an edge in a random direction, but an **RRT** first selects a goal point, then tries to add an edge from the closest node in the tree toward the goal point. **RRT*** improves on this by rewiring the tree to form shortest paths

 A tree(**RT**) generated by random motion from a randomly selected tree node does not explore very far.
 A rapidly exploring random tree (**RRT**) first selects a goal point, then tries to add an edge from the closest node in the tree  toward the goal point. This results in a tree that tends to quickly explore the space, because search is biased into the largest Voronoi regions of a graph defined by the tree. 
 **RRT*** improves on this by rewiring the tree to form shortest paths. RRT* converges to the shortest path, at the cost of more computation. 
 
 All three trees are probabilistically complete, meaning if a nonzero width path exists, the tree will eventually find the path. 
 
* [A* Pathfinding (E01: algorithm explanation), by Sebastian Lague](https://youtu.be/-L-WgKMFuhE)
 
 ![](https://i.imgur.com/LVWrO7E.png)


G cost: Left Corner  
H cost: Right Corner  
F cost: Middle one  
We choose the one which has low F cost from the explored nodes. And calculate values for that node's surrounding nodes.And keep repeating the process. 

![](https://i.imgur.com/PwesfsV.png)


If at times the F cost comes out to be same we consider the lowest H cost first. But also explore all other nodes as well with the same F cost.  

GREEN: Explored Nodes  
RED: Selected minimum F cost nodes.  

Examples

![](https://i.imgur.com/lAQ8w7e.png)

![](https://i.imgur.com/NgVbvmu.png)



**Psuedo Code**

![](https://i.imgur.com/DtGeZzA.png)





