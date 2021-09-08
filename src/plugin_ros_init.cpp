#include "gazebo/rendering/rendering.hh"
#include "gazebo/gazebo.hh"
#include "ros/ros.h"

namespace gazebo
{
class GazeboROSInit : public SystemPlugin
{
protected:
    boost::shared_ptr<ros::NodeHandle> nh_;
    boost::shared_ptr<ros::AsyncSpinner> async_ros_spin_;
    bool stop_;
public: 
    virtual ~GazeboROSInit(){
        stop_ = false;
        async_ros_spin_->stop();
        nh_->shutdown();
    }
    
    
    virtual void Load(int argc, char ** argv){
        ROS_INFO("GazeboROSInit system plugin has been loaded!");
        gazebo::event::Events::ConnectSigInt(boost::bind(&GazeboROSInit::shutdownSignal,this));
        //setup ROS
        if(!ros::isInitialized())
            ros::init(argc,argv,"gazebo", ros::init_options::NoSigintHandler);
        else
            ROS_ERROR("ROS has not been initialized in gazebo system plugin!") ;
        
        while(!ros::master::check()){
            ROS_WARN_STREAM_NAMED("gazebo_ros_init","No ROS master - start roscore to continue...");
            usleep(500*1000);
            
            if(stop_){
                ROS_WARN_STREAM_NAMED("gazebo_ros_init","Canceled ROS initialization by sigint event");
                return;
            }
        }
        
        nh_.reset(new ros::NodeHandle("~"));
        
        // Built-in multi-threaded ROS spinning
        async_ros_spin_.reset(new ros::AsyncSpinner(0)); // will use a thread for each CPU core
        async_ros_spin_->start();        
        
    }
    
    void shutdownSignal(){
        stop_ = true;
    }

private: 
    virtual void Init() {
    }
};

// Register this plugin with the simulator
GZ_REGISTER_SYSTEM_PLUGIN(GazeboROSInit)
}