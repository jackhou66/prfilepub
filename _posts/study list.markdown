
ros robot  collision check algrithem (for TTC)

https://mr-winter.blog.csdn.net/article/details/141402617

https://github.com/aws-robotics/aws-robomaker-small-warehouse-world/fork    ros2 

Dataset-of-Gazebo-Worlds-Models-and-Maps
https://github.com/mlherd/Dataset-of-Gazebo-Worlds-Models-and-Maps  


path planning   and  Trajectory planning 
![alt text](path.png)
https://mr-winter.blog.csdn.net/article/details/128892963 
https://github.com/ai-winter/python_motion_planning


main code;
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

RRT 
https://github.com/ai-winter/ros_motion_planning



Map viz  similar google 

Desktop$ git clone -b foxy-eol  https://github.com/swri-robotics/mapviz.git


ros2  humble
https://github.com/foxglove/ros-foxglove-bridge
