
#include <sensor_msgs/LaserScan.h>
#include <laser_geometry/laser_geometry.h>
#include <tf/transform_listener.h>
#include <std_srvs/Empty.h>
#include <boost/thread/recursive_mutex.hpp>
#include <std_msgs/Float32.h>

class ClosestPoint
{
public:
    ClosestPoint(ros::NodeHandle& nh_priv):
    _nh_priv(nh_priv),
    _nb_scans_init(20),
    _status(ClosestPoint::FIRST_CALLBACK),_current_scan_idx(0)
    {
        std::string scan_in_topic("/laserscan"),
                    scan_out_topic("/laserscan_min");
        _nh_priv.getParam("scan_in_topic",scan_in_topic);
        _nh_priv.getParam("scan_out_topic",scan_out_topic);
        
        _nh_priv.getParam("nb_scans_init",_nb_scans_init);
        
        std::string min_dist_out_topic("/min_dist_to_laser");
        _nh_priv.getParam("min_dist_out_topic",min_dist_out_topic);
        
        _scan_sub = _nh_priv.subscribe(scan_in_topic,10,&ClosestPoint::findClosestCallback,this);
        _scan_pub = _nh_priv.advertise<sensor_msgs::LaserScan>(scan_out_topic,10);
        _min_dist_pub = _nh_priv.advertise<std_msgs::Float32>(min_dist_out_topic,10);
    }
    void findClosestCallback(const sensor_msgs::LaserScan::ConstPtr& scan_in);
    bool reset(std_srvs::EmptyRequest& ,std_srvs::EmptyResponse& );
protected:
    ros::Subscriber _scan_sub;
    ros::Publisher _scan_pub;
    ros::Publisher _min_dist_pub;
    sensor_msgs::LaserScan _scan_min;
    std::vector<float> _mask;
    unsigned int _current_scan_idx;
    int _nb_scans_init;
    laser_geometry::LaserProjection _projector;
    tf::TransformListener _tf;
    ros::NodeHandle _nh_priv;
    enum STATUS{FIRST_CALLBACK,LEARNING,PROCESSING};
    STATUS _status;
    boost::recursive_mutex _mutex;
};









