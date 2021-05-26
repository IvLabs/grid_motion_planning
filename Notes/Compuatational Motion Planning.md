# [Compuatational Motion Planning](https://www.coursera.org/learn/robotics-motion-planning/home/welcome)  

## Week 1
* The goal is to develop techniques that will allow a robot or robots to automatically decide on how to move from one position or configuration to another. Often we are interested in finding a path that minimizes some cost or distance metric.
* https://nrsyed.com/2017/12/30/animating-the-grassfire-path-planning-algorithm/  


### Grassfire Algorithm
*  We will begin by marking the destination node with a distance value of 0.Then, we find all of the nodes that are 1 step away from the destination, and mark them with a 1. Then, all the nodes that are 2 steps away, mark them with a 2.. All of the nodes that are 3 steps away, mark with a 3, etc, etc, etc, until we encounter the start node.
*  To move towards the destination from the starting node, move towards the neighbour with smallest distance value, breaking ties arbitarily.
*  If the start node has not been marked, there is no path from the start to the destination.

**GrassFire**
![**GrassFire**](https://i.imgur.com/FxWoyLf.jpg)

**Pseudo Code**
![](https://i.imgur.com/pwT9xoK.jpg)


The amount of computational effort that we'd need to expend in order to run the grassfire algorithm on a grid grows linearly with the number of nodes.  
>O(|V|)
>V is number of nodes.

*  If a path exist between the start and the destination node, it will find one with the fewest number of edges.
* If no path exists, the algorithm will discover that fact and report it to the user.

### Dijkstra Algorithm
* We begin by marking our starting node with a distance value of zero. Here we use a red color to indicate that this node has been visited or marked. We then mark each of the nodes that are adjacent to node A, in this case, nodes B, D, and F. We use the blue color to indicate that these notes are now part of a list that we're currently considering.
* Then from these considered noodes we chose the one with smallest distance and marks each of its adjacent nodes with the distance.
* After this we visit the node with smallest distance from all considered nodesand repeat the process untill we reach the goal destination.

**Dijkstra**
![](https://i.imgur.com/Mnkn1ye.jpg)

**Pseudo Code**
![](https://i.imgur.com/cDVsuOm.jpg)

 The computation complexity of this algorithm is related to the number of nodes and the number of edges in the graph.
 >O(|V^2|)
 
V is number of nodes.
As a priority queue, we can actually reduce the computational complexity to a term that grows more slowly, as shown in the equation,
>O((|E|+|V|)log(|V|))

E is the number of edges.

Dijkstra's procedure is a bit more expensive than grass fire but only by a term that grows logarithmically in the number of nodes.

* The basic reason that Dijkstra's algorithm works is that we end up marking nodes based on their distance from the start node.That is whenever we mark a node, color it red in our example, we can be sure that there is no shorter path to that node from the start node via any of the nodes that we haven't marked yet,given the fact that our edge weights are in fact non-negative. It successfully find the shortest path when exits and records failure when not.


### A* Algorithm

**Heurestic Function Creteria**
* The heuristic function H(n) serves as an underestimate for the distance between the node n and the goal.
*  H applied to the goal node returns as 0  
H(Goal)=0
*  For any two nodes, x and y, that are adjacent in the graph, 
H(x) <= H(y) + d(x,y)
Where d(x,y) denotes the length of the edge between those two nodes x and y.
* H(n)<= the lenghth of shortest path from goal to n.

 **Heurestic Functions**
![](https://i.imgur.com/g2mwuTO.jpg)

**Pseudo Code**
![](https://i.imgur.com/LpJMp4Z.jpg)

* We associate two numerical attributes with each of the nodes in our graph g and h. g the distance from start to current node and h the distance from current node to end.  
f value is the sum of the node g value and the heuristic cost(h) associated with that node.
>f=h+g
* In other words, f the sum of the distance between the node and the start and the estimate for how much distance is left to go between that node and the goal location.
* On every iteration of this algorithm, the system chooses the node with the smallest associated f value. That is, it attempts to choose the node that is most likely to be on the shortest path between the start and the goal node.
*  Once again, once a node is chosen, the f and g values associated with each of its neighbors are updated. In other words, on each iteration, we're trying to pick the node that is most likely to be on the shortest path from the start to the destination. 
*  We then update the neighbors of those chosen node and repeat the process until we encounter the goal node or run out of nodes to consider. 


## Week 2

### Configuration Space Applications

#### Introduction to Configuration Space

* In the real world, most of the robots we are going to build can move continuously through space. The configuration space of a robot is the set of all configurations and/or positions that the robot can attain.
* Here we can quantify the positions that the robot can take on with a tuple composed of two numbers, tx and ty, which denote the coordinates of a particular reference point on the robot, with respect to a fixed coordinate frame of reference.
![](https://i.imgur.com/EoKxsPk.png)
* Here are a couple of configurations that this translating robot can take on, along with the associated coordinates.
![](https://i.imgur.com/7YxpBA1.png) 
![](https://i.imgur.com/k8YOh2u.png)

**Adding an Obstacle in the to the space**
![](https://i.imgur.com/S3mPKcz.png)

* The configuration space of a robot is the set of all configurations and/or positions that the robot can attain.
* Image given below shows a simple example of a robot that can translate freely in the plane.
 ![](https://i.imgur.com/cLvwASe.png)
 
* In this case, our robot has 2 degrees of freedom, and we can associate the configuration space of the robot with the points on the 2D plane.
* Obstacles make certain configurations in the configuration space unattainable.
* This set of configurations that the robot cannot inhabit is referred to as a configuration space obstacle.

![](https://i.imgur.com/pqvc6LY.png)
* The region of configuration space that the robot can attain is referred to as the free space of the robot. 
* On the right-hand side of the above figure, we plot the configuration space obstacle, corresponding to the geometric obstacle shown in the left side of the figure.
* The dimensions and shape of the configuration space obstacle are obtained by considering both the obstacle and the shape of the robot.
* More formally, the configuration space obstacle is defined by what's known as the [Minkowski sum](https://doc.cgal.org/latest/Minkowski_sum_2/index.html) of the obstacle and the robot shape.
* For multiple obstacles, we have to consider union of all of the configuration space obstacles.

  ![](https://i.imgur.com/vY1xdj8.png)
* The dark areas correspond to configurations that the robot cannot attain.

#### Translating and Rotating Robot

![](https://i.imgur.com/px4Iyzc.jpg)
* Our robot now has three degrees of freedom.
* We can denote the configuration of our robot with a tuple, tx, ty, and theta, where tx and ty still denote the position of a reference point in the plane, and theta denotes the applied rotational angle in degrees.
* When we introduce obstacles into the workspace, we can think about the set of configurations that are limited.
> In this case, the configuration space has three dimensions, and the configuration space obstacles can be thought of as three dimensional regions in this space.

**3D Obstacle and its Configuratio Space**

![](https://i.imgur.com/E3n7u6N.jpg)

![](https://i.imgur.com/qHmC2aS.jpg)
* The vertical access corresponds to the rotation theta, while the other two horizontal axes correspond to the translational parameters tx and ty. In the second figure, the surface that we are visualizing corresponds to the surface of the configuration space obstacle.
* As before, the basic problem in motion planning is to come up with a trajectory between a start point and an end point that avoids all the configuration space obstacles 
> For 3D visualisation of Configuration Space [go here](https://drive.google.com/file/d/1u5K_w101scf8WaOBiSrbAFn7bgXWUhq8/view?usp=sharing)

### Path Planning in Configuration Space

**Visibility Graph**

* Our goal is to come up with a trajectory from the starting two deconfiguration to the ending configuration. In this case, the configuration space obstacles are modeled as polygons, which suggests the following discretization.
* We associate a node with every configuration space obstacle vertex, as shown below and we draw an edge between any two vertices that can be connected by a straight line that lies entirely in free space.
* This is known as a visibility graph.
![](https://d3c33hcgiwev3.cloudfront.net/imageAssetProxy.v1/d5QUlZolEeeVtQqr-Nx1og_368d2760c03b0b2893080226ec2b0ec7_Image.png?expiry=1621468800000&hmac=ZelgkKR_BoXecZ7nfuidi-cuXdRVenaqokZz8X7RKVQ)
> The blue lines don't have any special meaning the peeps at Penn just forgot to draw them.

* Once we've done this, we construct the shortest path through the graph between the start node and the end node. A problem that can be readily solved using Dijkstra's algorithm.

![](https://i.imgur.com/lx4seVh.jpg)

**Trapezoidal Decomposition**
![](https://i.imgur.com/BuVKroj.jpg)
* We divide the robot's free space into a set of simpler regions and then form a graph where the nodes of the regions and the edges indicate which regions are adjacent to each other.
* A common approach to constructing such a decomposition for a two-dimensional configuration space is that we sort the obstacle vertices based on their x-coordinates, and proceed from left to right, dividing the free space up into regions as we proceed.
* We can then form a graph where the nodes of these trapezoidal regions of free space and the edges indicate which of these regions are adjacent to each other. Path planning is then carried out by finding out which cell contains the start location and which the goal. And then planning a path through the graph between these two nodes as shown.

**Collision Detection and Freespace Sampling Methods**

*Collision Detection Function*
1. Let x denote the coordinates of a point in configuration space.
2.  We define a function, let's call it CollisionCheck, that returns zero if the configuration is in freespace, and one otherwise i.e. if there is obstacle it would return one.
3.  For this we represent both our robot and our obstacle as collections of triangles, as shown in this figure.
![](https://i.imgur.com/ebpnBW2.jpg)
4.  We can test where the two triangles intersect by checking all of the sides on both triangles, and testing whether any of those sides act as separating lines, where all of the points from one triangle lie on one side of the line, or not. If you can find any separating edge, among the six edges among the two triangles, you have proved that the triangles don't intersect.


## Week 3
### Sampling-based Planning Methods
#### Probabilistic Road Maps (PRM)

* The approach of discreetizing the configuration space evenly on a grid, can work well when the dimension of the space is small. But the number of samples required can grow to be frighteningly large as we increase the dimension of the space.
 ![](https://i.imgur.com/27etSB3.jpg)
    1. On every iteration, the system chooses a configuration in the configuration space at random, and tests whether it is in free space using the collision check function. The new random node is the green dot.
    2. If it is in free space, it then tries to see if it can forge routes between this new configuration and the closest existing samples in the graph.Every path that it creates is recorded as a new edge in the graph that the system is building.
    3. The solid green lines correspond to new links that are added, while the dashed green line represents a connection that failed due to collision with the obstacle.
    4. We call this procedure the Probabilistic Road Map method, or PRM for short. Probabilistic to reflect the stochastic nature of the process and road map for the graph that the procedure constructs which we hope will serve as a road map for the freespace.  
    
**Pseudo Code**
![](https://i.imgur.com/N6PYJln.jpg)

This procedure requires two ingredients
1. Distance Function
    * Manhattan Distance 
    * Euclidian Distance
    1. There are often cases where some of the coordinates of the configuration space correspond to angular rotations. In these situations care must be taken to ensure that the Dist function correctly reflects distances in the presence of wraparound
    2. For example if theta1 and theta2 denote two angles between 0 and 360 degrees the expression below can be used to capture the angular displacement between them.  
     Dist(theta1, theta2,) = min(|theta1 - theta2|,(360 - |theta1 - theta2|))

2. Local Planner
    * It decides whether there is a path between two points or not. A common way to handle this is to construct a set of evenly spaced samples on a straight line between the two configurations. And to use the collision check function to check that all of these intermediate configurations are collision free.

* Once the PRM procedure has generated what we believe to be a sufficient sampling of the configuration space, we can try to generate paths between designated start and end configurations. 
* If this step succeeds, one can then attempt to find a path between the start and end nodes via the road map using our usual suite of graph-based planning algorithms, like Dijkstra's method or A*. 

**Issues with PRM**

* Not strictly speaking complete. 
With the PRM procedure, it is possible to have a situation where the algorithm would fail to find a path even when one exists. 
Thus the behavior of these sampling based algorithms are probabilistically complete.
Thus, if the procedure doesn't find a path, it's hard to know whether there is in fact no path, or whether you would be able to find a way if you kept trying long enough. 

* Solution to these issues
    * Try to sample more points closer to the boundaries of configuration space obstacles. In the hopes of constructing path that skirt the surfaces.
    * In Practice no path planning problem is pathological and generally uniform random sampling is a good place to start
    * Since the samples are choosing randomly the resulting trajectory might not be optimal and maybe jerky.

* A real advantage of these PRM based planners, is that they can be applied to systems with lots of degrees of freedom, which is definately a big advantage over grid - bases planners.  

#### Rapidly Exploring Random Trees
> Procedures that **explicitly** consider the start and the goal locations in the sampling procedure.
1. This approach works by generating random samples and connecting them together to form a  special kind of graph where each where every node is connected to a single parent and the tree is rooted at a given starting location

**Pseudo code**
![](https://i.imgur.com/yId3jCU.png)
2. Here the red node depicts the new random configuration that the system generates, while y depicts the closest existing node in the tree. Now a new node z, which is generated by finding a configuration that is distance delta away from y along the line towards x. Next we see the new state of the tree after adding the node z. If the procedure does not succeed in this process of stepping towards a random node, it's simply abandons a point and moves on to the next iteration where it will generate a new random sample and try again.

![](https://i.imgur.com/CwVLAK7.jpg)
>Dist(y,z) = delta

3. To construct a path between the start configuration and an end configuration, we actually construct two trees. One rooted at the start location and one at the goal.

![](https://i.imgur.com/JpFl5vM.png)


**RRT 2 Trees Pseudo Code**
![](https://i.imgur.com/KjYeNMx.jpg)

*  Like PRM, RRT algorithm is also probabilisticly complete. So, there's a certain probability that the method will find the path from the start to the goal if one exists. In practice, the RRT Method is very effective at forging paths in high-dimensional configuration spaces.

## Week 4
### Planning with Artificial Potential Fields
1. Constructing an Artificial Force Field
    * We construct a smooth function over the extent of the configuration space, which has high values when the robot is near to an obstacle and lower values when it's further away, with the lowest value at the goal.  
    * We use the gradient of the funtion to guide the robot.
    * One of the simplest way to create a ***Attractive Potential Field***(***F~a~***) is to take the quadratic function
    ![](https://i.imgur.com/5vWHeIb.png)
    ![](https://i.imgur.com/MXQE7vE.png)
    * In addition to attracting to goal we want to repel obstacles, so we create a ***Repulsive Potential Field***(***F~r~***)
    * Let ***ρ(x)*** denote a function that takes as input the coordinates of a point in our two dimensional configuration space and returns the distance to the closest configuration space obstacle.
    * If we have explicit representations of our configuration space obstacles, we can sometimes construct such a function analytically.
    * If we discretized our two dimensional work space into a grid, we can apply an operation from image processing called [Distance Transformation](https://drive.google.com/file/d/1FqhBcd3DQipp0r4d9XPKONg3KG8fg2gk/view?usp=sharing) to compute this distance at least approximately at every grid location.
    ![](https://i.imgur.com/1qkDx9V.png)
    * In ***F~r~***(***x***) the parameter d~0~ controls the influence of the function.If the distance between the robot and the obstacle is greater than d~0~, the function shuts off, and when the robot approaches the obstacles, then it increases rapidly.
    ![](https://i.imgur.com/cBi2pA3.png)
    
    * Here is a surface plot, showing the sum of the two functions.
    ![](https://i.imgur.com/E8GSjr1.png)
    * Next we use the gradient of the function to steer the robot.
    * More specifically, we choose the velocity of the robot v based on the gradient of the potential function as below.
    ![](https://i.imgur.com/h4sTpu7.png)
    * The proportionality sign here indicates that the gradient will be used to decide on the direction of motion, but the speed of the robot, which corresponds to the magnitude of the velocity vector is up to the programmer.
    * To summarize the algorithm, the robot continually evaluates the gradient of the artificial potential field and steps in that direction until it is close enough to the desired goal.
    *  A quiver plot, is an alternate method to depict the same information, where the arrows indicate the direction of the gradient field at various locations in the configuration space.
    *  This is referred to as Sensor Based Planning. The readings from the robot's position sensor are used to pull it towards the goal. While the measurements from it's range sensors like laser scanners or stereo systems are used to push it away from the obstacles in the environment.
    ![](https://i.imgur.com/66T1UAR.png)


### Related Issues
*  It can be very difficult to ensure that they will always work. Ideally, our artificial potential function would only have a single global minimum located at the desired configuration. In practice, you can run into situations where the attractive and repulsive forces conspire to produce local minimum at locations other than the desired location. Resulting into two minimums. For example for the configuration space given below, when we create a Artificial Potential Field we get a surface like this. Here we have 2 local minima one at the goal and one at the cul-de-sac here.
![](https://i.imgur.com/Hb7RKJf.png)
![](https://i.imgur.com/l7xGuMx.png)

*  Thus this may or may not converge the goal depending upon where the robot starts.And converges to the local minima instead of the goal.
*  One way to view these artificial potential field based schemes is as a useful heuristic. In many cases, they will successfully guide the robot to the desired configuration, but they can get stuck in dead ends, which often necessitates the use of a back tracking procedure to detect these situations and to switch to a different planning strategy.

###  Generalizing Potential Fields
![](https://i.imgur.com/wivAeFi.png)
![](https://i.imgur.com/x1Mbeqd.png)
* One way to visualize that each of these control points is outfitted with a proximity sensor which it can use to detect the range to nearby obstacles. The artificial potential field uses this information to push each of these control points away from obstacles while guiding them towards their desired goals.
* The effect of all of these pushes and pulls is arrogated in the artificial potential field and the gradient information is used to decide how to move locally.


