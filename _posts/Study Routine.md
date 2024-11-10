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



![](/img/path2.png)

** path **

![](/img/path3.png)


A[GPS map](https://github.com/yanliang-wang/path_api_display)
