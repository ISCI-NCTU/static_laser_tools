#include <static_laser_tools/static_laser_tools.hpp>

bool ClosestPoint::scanContours(std_srvs::SetBoolRequest& req,std_srvs::SetBoolResponse& resp)
{
    boost::recursive_mutex::scoped_lock lock(_mutex);
    _current_scan_idx = 0;

    if(req.data)
    {
        _status = INITIALIZATION;
        for(int i = 0;i<_mask.size();i++)
            _mask[i] = std::numeric_limits<double>::quiet_NaN();
    }else
    {
        _status = LEARNING;
    }
    resp.success = true;
    return true;
}

void ClosestPoint::findClosestCallback(const sensor_msgs::LaserScan::ConstPtr& scan_in)
{
    double min_dist = 1e6;
    bool has_min = false;
    boost::recursive_mutex::scoped_lock lock(_mutex);
    _scan_min = *scan_in;
    switch(_status)
    {
        case INITIALIZATION:
        {
            ROS_INFO("-- First callback, copying data..");
           _mask = scan_in->ranges;
           _status = LEARNING;
        }break;

        case LEARNING:
        {
            ROS_INFO("-- Learning %d/%d",_current_scan_idx,_nb_scans_init);
            if(++_current_scan_idx > _nb_scans_init)
            {
                ROS_INFO("-- Processing...");
                _current_scan_idx = 0;
                _status = PROCESSING;
                break;
            }
            for(int i=0;i<scan_in->ranges.size();i++)
            {
                if(std::isnan(_mask[i]) ||
                    ( !std::isnan(scan_in->ranges[i]) &&
                    scan_in->ranges[i] <= _mask[i]))
                {
                    _mask[i] = scan_in->ranges[i] - _final_offset;
                }
            }
            for(int i=0;i<scan_in->ranges.size();i++)
                _scan_min.ranges[i] = _mask[i];
        }break;

        case PROCESSING:
        {
            for(int i=0;i<_scan_min.ranges.size();i++)
            {
                if(!std::isnan(scan_in->ranges[i]) &&
                    scan_in->ranges[i] < _mask[i])
                {
                    _scan_min.ranges[i] = scan_in->ranges[i];
                }else
                {
                    _scan_min.ranges[i] = std::numeric_limits<double>::quiet_NaN();
                }
                if(_scan_min.ranges[i] < min_dist){
                    min_dist = _scan_min.ranges[i];
                    has_min = true;
                }
            }
            if(has_min)
            {
              std_msgs::Float32 d;
              d.data = min_dist;
              _min_dist_pub.publish(d);
            }
        }break;

        default:
            break;
    }
    sensor_msgs::LaserScan mask;
    mask = *scan_in;
    for(int i=0;i<scan_in->ranges.size();i++)
        mask.ranges[i] = _mask[i];

    _mask_pub.publish(mask);
    _scan_pub.publish(_scan_min);
}
