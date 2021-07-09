#include "OctoTerrainMap.h"


OctoTerrainMap::OctoTerrainMap(costmap_2d::Costmap2D *occ_grid){
    ros::NodeHandle nh;
    ros::Rate loop_rate(10);
    ros::ServiceClient client = nh.serviceClient<octomap_msgs::GetOctomap>("/rtabmap/octomap_full"); 
    octomap_msgs::GetOctomap srv;

    client.waitForExistence();
    
    ROS_INFO("Requesting Octomap");
    if(client.call(srv)){
        ROS_INFO("Got the full octomap");
    }
    else{
        ROS_INFO("Could not obtain an octomap");
    }
    octomap_ = (octomap::OcTree*) octomap_msgs::fullMsgToMap(srv.response.map);
    
    /*
    ros::ServiceClient grid_client = nh.serviceClient<rtabmap_ros::GetMap>("/rtabmap/get_map_data");
    rtabmap_ros::GetMap grid_srv;
    grid_srv.request.global = true;
    grid_srv.request.optimized = true;
    grid_srv.request.graphOnly = true;
    
    grid_client.waitForExistence();
    if(grid_client.call(grid_srv)){
        ROS_INFO("Got the occupancy grid");
    }
    else{
        ROS_INFO("Did not get the occupancy grid");
    }
    */
    
    occ_grid_ = occ_grid;
    cols_ = occ_grid_->getSizeInCellsX();
    rows_ = occ_grid_->getSizeInCellsY();
    map_res_ = occ_grid->getResolution();
    
}


OctoTerrainMap::~OctoTerrainMap(){
    delete octomap_;
}


float OctoTerrainMap::getMapRes(){
    return map_res_;
}






//overriden methods

BekkerData OctoTerrainMap::getSoilDataAt(float x, float y) const{
    return lookup_soil_table(0);
}

float OctoTerrainMap::getAltitude(float x, float y, float z_guess) const{
    octomap::point3d start(x, y, z_guess);
    octomap::point3d direction(0,0,-1);
    octomap::point3d end;
    
    octomap_->castRay(start, direction, end);
    
    return end.z();
}

int OctoTerrainMap::isStateValid(float x, float y) const{
    //look up in the oc_grid_ to see if thing is occupied
    unsigned mx;
    unsigned my;
    
    if(!occ_grid_->worldToMap(x, y, mx, my)){
        return 0; //Out of Bounds
    }
    
    if(occ_grid_->getCost(mx, my) == costmap_2d::LETHAL_OBSTACLE){
        return 0; //state is occupied if occupancy > 50%. At least I think thats how it all works.
    }
    
    return (x > Xmin) && (x < Xmax) && (y > Ymin) && (y < Ymax);
}

std::vector<Rectangle*> OctoTerrainMap::getObstacles() const{
    std::vector<Rectangle*> obstacles; //Lol, idk what I'm gonna do here exactly. I might remove this method from the base class
    return obstacles;
}
