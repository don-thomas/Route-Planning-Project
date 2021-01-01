#include "route_planner.h"
#include <algorithm>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

    // Store start_node and end_node attributes in the RoutePlanner's 
  	this->start_node = &m_Model.FindClosestNode(start_x, start_y);
  	this->end_node = &m_Model.FindClosestNode(end_x, end_y);

}



float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
	float  h_distance =  node->distance(*end_node);
  	return h_distance;
}


void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
	(*current_node).FindNeighbors(); // Finds neighbor nodes
  	
  	//Vector of pointers of the neighbor nodes
  	std::vector<RouteModel::Node *> present_neighbors = (*current_node).neighbors;
  	
  
  	for(auto neighbor_node : present_neighbors) {
      
      // Dont Search In Visited Nodes
      if( (*neighbor_node).visited == true ){
        continue; // Look at the next node
      }
      
      float neighbor_g = (*current_node).g_value + current_node->distance(*neighbor_node);
      float neighbor_h = RoutePlanner::CalculateHValue(neighbor_node);
      
      (*neighbor_node).visited = true;
      (*neighbor_node).parent = current_node;
      (*neighbor_node).h_value = neighbor_h;
      (*neighbor_node).g_value = neighbor_g;
      
      this->open_list.push_back(neighbor_node);
    }
}

// Function needed for the sort vector function
bool Compare(const RouteModel::Node* a, const RouteModel::Node* b) {
  float f1 = (*a).h_value + (*a).g_value; // f1 = g1 + h1
  float f2 = (*b).h_value + (*b).g_value; // f2 = g2 + h2
  return f1 > f2; 
}


RouteModel::Node *RoutePlanner::NextNode() {
	std::sort((*this).open_list.begin(), (*this).open_list.end(), Compare);
  
  	RouteModel::Node* nextnode = this->open_list.back();
  	this->open_list.pop_back();
  	return nextnode;
}



std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    distance = 0;
    std::vector<RouteModel::Node> path_found;
	
  	RouteModel::Node *back_step_node = current_node;
   	
  	// Start node has a parent with nullptr
    while(back_step_node->parent != nullptr){ 
  		path_found.push_back(*back_step_node);
      	distance += back_step_node->distance(*(back_step_node->parent));
      	back_step_node = back_step_node->parent;
    }
  	path_found.push_back(*back_step_node);
  	std::reverse(path_found.begin(),path_found.end());

    distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.
    return path_found;

}



void RoutePlanner::AStarSearch() {
    RouteModel::Node *current_node = this->start_node;
  	(*current_node).visited = true;
  	(*current_node).g_value = 0;
  	(*current_node).h_value = RoutePlanner::CalculateHValue(current_node);
  	this->open_list.push_back(current_node);
  
  	while(current_node != end_node){
      	current_node = this->NextNode();
     	this->AddNeighbors(current_node);
      	
    }
	std::vector<RouteModel::Node> finalpath = this->ConstructFinalPath(current_node);
  
  	this->m_Model.path = finalpath; // Returns the correct path
}