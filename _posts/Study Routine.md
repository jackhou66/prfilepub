---
layout: post
title: " Beginner's mind  "
subtitle: "It's wonderful to have a beginner's mind "
author: "Hou"
header-img: "img/event.jpg"
header-mask: 0.2
tags:
  - Path planning
  - Flash
---

![](/img/path.png)
 
<br/>

A[ros_motion_planning](https://github.com/ai-winter/ros_motion_planning)；

![](/img/demo.gif)

**motion_planning**

<br/>

 ** path **

![](/img/path_aigri.png)  

 ** python  **
def plan(self):
     # OPEN set with priority and CLOSED set
     OPEN = []
     heapq.heappush(OPEN, self.start)
     CLOSED = []

     while OPEN:
         node = heapq.heappop(OPEN)

         # exists in CLOSED set
         if node in CLOSED:
             continue

         # goal found
         if node == self.goal:
             CLOSED.append(node)
             return self.extractPath(CLOSED), CLOSED

         for node_n in self.getNeighbor(node):                
             # exists in CLOSED set
             if node_n in CLOSED:
                 continue
             
             node_n.parent = node.current
             node_n.h = self.h(node_n, self.goal)

             # goal found
             if node_n == self.goal:
                 heapq.heappush(OPEN, node_n)
                 break
             
             # update OPEN set
             heapq.heappush(OPEN, node_n)
         
         CLOSED.append(node)
     return [], []


** C++ **

std::tuple<bool, std::vector<Node>> AStar::plan(const unsigned char* costs, const Node& start,
                                                  const Node& goal, std::vector<Node> &expand) {
    // open list
    std::priority_queue<Node, std::vector<Node>, compare_cost> open_list;
    open_list.push(start);

    // closed list
    std::unordered_set<Node, NodeIdAsHash, compare_coordinates> closed_list;

    // expand list
    expand.clear();
    expand.push_back(start);

    // get all possible motions
    const std::vector<Node> motion = getMotion();

    // main loop
    while (!open_list.empty()) {
      // pop current node from open list
      Node current = open_list.top();
      open_list.pop();
      current.id = this->grid2Index(current.x, current.y);
      
      // current node do not exist in closed list
      if (closed_list.find(current) != closed_list.end())
        continue;

      // goal found
      if (current==goal) {
        closed_list.insert(current);
        return {true, this->_convertClosedListToPath(closed_list, start, goal)};
      }

      // explore neighbor of current node
      for (const auto& m : motion) {
        Node new_point = current + m;

        // current node do not exist in closed list
        if (closed_list.find(new_point) != closed_list.end())
          continue;

        // explore a new node
        new_point.id = this->grid2Index(new_point.x, new_point.y);
        new_point.pid = current.id;

        // if using dijkstra implementation, do not consider heuristics cost
        if(!this->is_dijkstra_)
          new_point.h_cost = std::sqrt(std::pow(new_point.x - goal.x, 2)
                              + std::pow(new_point.y - goal.y, 2));

        // if using GBFS implementation, only consider heuristics cost
        if(this->is_gbfs_)
          new_point.cost = 0;
        
        // goal found
        if (new_point==goal) {
          open_list.push(new_point);
          break;
        }

        // bext node hit the boundary or obstacle
        if (new_point.id < 0 || new_point.id >= this->ns_ || 
            costs[new_point.id] >= this->lethal_cost_ * this->factor_)
            continue;

        open_list.push(new_point);
        expand.push_back(new_point);
      }
      closed_list.insert(current);
    }
    return {false, {}};
  }
}

** RRT **

![](/img/RRT.png)

import math
import os
import sys

import matplotlib.pyplot as plt
from celluloid import Camera  # 保存动图时用，pip install celluloid
sys.path.append("../RRT")
try:
    from rrt_planning import RRT
except ImportError:
    raise

show_animation = True


class RRTStar(RRT):
    """
    Class for RRT Star planning
    """

    class Node(RRT.Node):
        def __init__(self, x, y):
            super().__init__(x, y)
            self.cost = 0.0

    def __init__(self,
            start,
            goal,
            obstacle_list,
            rand_area,
            expand_dis=3.0,
            goal_sample_rate=20,
            max_iter=500,
            connect_circle_dist=50.0,
            search_until_max_iter=False,
            robot_radius=0.0):
        """
        Setting Parameter

        start:Start Position [x,y]
        goal:Goal Position [x,y]
        obstacleList:obstacle Positions [[x,y,size],...]
        randArea:Random Sampling Area [min,max]

        """
        super().__init__(start, goal, obstacle_list, rand_area, expand_dis,
                          goal_sample_rate, max_iter,
                         robot_radius=robot_radius)
        self.connect_circle_dist = connect_circle_dist
        self.goal_node = self.Node(goal[0], goal[1])
        self.search_until_max_iter = search_until_max_iter

    def planning(self, animation=True, camera=None):
        """
        rrt star path planning

        animation: flag for animation on or off .
        """

        self.node_list = [self.start]
        for i in range(self.max_iter):
            print("Iter:", i, ", number of nodes:", len(self.node_list))
            rnd = self.sample_free()
            nearest_ind = self.get_nearest_node_index(self.node_list, rnd)
            new_node = self.steer(self.node_list[nearest_ind], rnd,
                                  self.expand_dis)
            near_node = self.node_list[nearest_ind]
            # 计算代价,欧氏距离
            new_node.cost = near_node.cost + math.hypot(new_node.x-near_node.x, new_node.y-near_node.y)

            if self.obstacle_free(new_node, self.obstacle_list, self.robot_radius):
                near_inds = self.find_near_nodes(new_node) # 找到x_new的邻近节点
                node_with_updated_parent = self.choose_parent(new_node, near_inds) # 重新选择父节点
                # 如果父节点更新了
                if node_with_updated_parent:
                    # 重布线
                    self.rewire(node_with_updated_parent, near_inds)
                    self.node_list.append(node_with_updated_parent)
                else:
                    self.node_list.append(new_node)

            if animation and i % 5 ==0:
                self.draw_graph(rnd, camera)

            if ((not self.search_until_max_iter) and new_node):  # if reaches goal
                last_index = self.search_best_goal_node()
                if last_index is not None:
                    return self.generate_final_course(last_index)

        print("reached max iteration")

        last_index = self.search_best_goal_node()
        if last_index is not None:
            return self.generate_final_course(last_index)

        return None

    def choose_parent(self, new_node, near_inds):
        """
        在新产生的节点 $x_{new}$ 附近以定义的半径范围$r$内寻找所有的近邻节点 $X_{near}$，
        作为替换 $x_{new}$ 原始父节点 $x_{near}$ 的备选
	    我们需要依次计算起点到每个近邻节点 $X_{near}$ 的路径代价 加上近邻节点 $X_{near}$ 到 $x_{new}$ 的路径代价，
        取路径代价最小的近邻节点$x_{min}$作为 $x_{new}$ 新的父节点
        Arguments:
        --------
            new_node, Node
                randomly generated node with a path from its neared point
                There are not coalitions between this node and th tree.
            near_inds: list
                Indices of indices of the nodes what are near to new_node
        Returns.
        ------
            Node, a copy of new_node
        """
        if not near_inds:
            return None

        # search nearest cost in near_inds
        costs = []
        for i in near_inds:
            near_node = self.node_list[i]
            t_node = self.steer(near_node, new_node)
            if t_node and self.obstacle_free(t_node, self.obstacle_list, self.robot_radius):
                costs.append(self.calc_new_cost(near_node, new_node))
            else:
                costs.append(float("inf"))  # the cost of collision node
        min_cost = min(costs)

        if min_cost == float("inf"):
            print("There is no good path.(min_cost is inf)")
            return None

        min_ind = near_inds[costs.index(min_cost)]
        new_node = self.steer(self.node_list[min_ind], new_node)
        new_node.cost = min_cost

        return new_node

    def search_best_goal_node(self):
        dist_to_goal_list = [self.calc_dist_to_goal(n.x, n.y) for n in self.node_list]
        goal_inds = [
            dist_to_goal_list.index(i) for i in dist_to_goal_list
            if i <= self.expand_dis
        ]

        safe_goal_inds = []
        for goal_ind in goal_inds:
            t_node = self.steer(self.node_list[goal_ind], self.goal_node)
            if self.obstacle_free(t_node, self.obstacle_list, self.robot_radius):
                safe_goal_inds.append(goal_ind)

        if not safe_goal_inds:
            return None

        min_cost = min([self.node_list[i].cost for i in safe_goal_inds])
        for i in safe_goal_inds:
            if self.node_list[i].cost == min_cost:
                return i

        return None 

    def find_near_nodes(self, new_node):
        """
        1) defines a ball centered on new_node
        2) Returns all nodes of the three that are inside this ball
            Arguments:
            ---------
                new_node: Node
                    new randomly generated node, without collisions between
                    its nearest node
            Returns:
            -------
                list
                    List with the indices of the nodes inside the ball of
                    radius r
        """
        nnode = len(self.node_list) + 1
        r = self.connect_circle_dist * math.sqrt((math.log(nnode) / nnode))
        # if expand_dist exists, search vertices in a range no more than
        # expand_dist
        if hasattr(self, 'expand_dis'):
            r = min(r, self.expand_dis)
        dist_list = [(node.x - new_node.x)**2 + (node.y - new_node.y)**2
                     for node in self.node_list]
        near_inds = [dist_list.index(i) for i in dist_list if i <= r**2]
        return near_inds

    def rewire(self, new_node, near_inds):
        """
            For each node in near_inds, this will check if it is cheaper to
            arrive to them from new_node.
            In such a case, this will re-assign the parent of the nodes in
            near_inds to new_node.
            Parameters:
            ----------
                new_node, Node
                    Node randomly added which can be joined to the tree

                near_inds, list of uints
                    A list of indices of the self.new_node which contains
                    nodes within a circle of a given radius.
            Remark: parent is designated in choose_parent.

        """
        for i in near_inds:
            near_node = self.node_list[i]
            edge_node = self.steer(new_node, near_node)
            if not edge_node:
                continue
            edge_node.cost = self.calc_new_cost(new_node, near_node)

            no_collision = self.obstacle_free(
                edge_node, self.obstacle_list, self.robot_radius)
            improved_cost = near_node.cost > edge_node.cost

            if no_collision and improved_cost:
                near_node.x = edge_node.x
                near_node.y = edge_node.y
                near_node.cost = edge_node.cost
                near_node.path_x = edge_node.path_x
                near_node.path_y = edge_node.path_y
                near_node.parent = edge_node.parent
                self.propagate_cost_to_leaves(new_node)

    def calc_new_cost(self, from_node, to_node):
        d, _ = self.calc_distance_and_angle(from_node, to_node)
        return from_node.cost + d

    def propagate_cost_to_leaves(self, parent_node):

        for node in self.node_list:
            if node.parent == parent_node:
                node.cost = self.calc_new_cost(parent_node, node)
                self.propagate_cost_to_leaves(node)


def main():
    print("Start " )
    fig = plt.figure(1)

    camera = Camera(fig) # 保存动图时使用
    # camera = None # 不保存动图时，camara为None
    show_animation = True
    # ====Search Path with RRT====
    obstacle_list = [
        (5, 5, 1),
        (3, 6, 2),
        (3, 8, 2),
        (3, 10, 2),
        (7, 5, 2),
        (9, 5, 2),
        (8, 10, 1),
        (6, 12, 1),
    ]  # [x,y,size(radius)]

    # Set Initial parameters
    rrt_star = RRTStar(
        start=[0, 0],
        goal=[6, 10],
        rand_area=[-2, 15],
        obstacle_list=obstacle_list,
        expand_dis=3,
        robot_radius=0.8)
    path = rrt_star.planning(animation=show_animation,camera=camera)

    if path is None:
        print("Cannot find path")
    else:
        print("found path!!")

        # Draw final path
        if show_animation:
            rrt_star.draw_graph(camera=camera)
            plt.plot([x for (x, y) in path], [y for (x, y) in path], 'r--')
            plt.grid(True)
            if camera!=None:
                camera.snap()
                animation = camera.animate()
                animation.save('trajectory.gif')
    plt.show()


if __name__ == '__main__':

    main()

![](/img/RRT1.gif)

[click](https://github.com/CHH3213/chhRobotics_CPP)

![](/img/path2.png)

** path **

![](/img/path3.png)


A[GPS map](https://github.com/yanliang-wang/path_api_display)



Global Path Planning Algorithm - Dijkstra's Algorithm
Global Path Planning Algorithm - Ant Colony Algorithm
Global Path Planning Algorithm - Dynamic Planning Algorithm
Global Path Planning Algorithm - A* Algorithm
Local Path Planning Algorithms - Curve Interpolation
Local Path Planning Algorithm - Artificial Potential Field Method
Local Path Planning Algorithm - Bessel Curve Method
Local Path Planning Algorithm - B-Spline Curve Method
Local Path Planning Algorithm - DWA Algorithm
Sampling-based path planning algorithm - PRM
Sample-based path planning algorithm - RRT
Sample-based Path Planning Algorithm - RRT-Connect
Sample-based Path Planning Algorithm - RRT*
Path Planning - Dubins Curve Derivation (Vector Based Approach)
Path Planning - Dubins Curve Formula Summary (Geometry Based Approach)
Path Planning - ReedsShepp Curve Summary
Introduction to Vehicle Speed Planning

math material (https://github.com/CHH3213/Books.git)  

Reinforcement Learning (https://github.com/CHH3213/chhRL)