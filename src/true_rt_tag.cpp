#include <true_rt_tag/true_rt_tag.h>

TRUE_RT_TAG::TRUE_RT_TAG(ros::NodeHandle* nh, ros::Rate rate): _nh(nh), _rate(rate) 
{
    package_path = ros::package::getPath("true_rt_tag");

    true_rt_tag_pub = _nh->advertise<geometry_msgs::PoseStamped>("true_rt_tag_pose", 1);

    tag_detections_sub = _nh->subscribe("/tag_detections", 1, &TRUE_RT_TAG::tag_detections_cb, this);
    // tf_static_sub = _nh->subscribe("/tf_static", 1, &TRUE_RT_TAG::tf_static_cb, this);

    _nh->getParam(ros::this_node::getName()+ "/tag_config_name", tag_config_name);

    if(loadTagConfig(tag_config_name))
        initialized = true;


    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
    try{
        cam2optical_geo = tfBuffer.lookupTransform("camera_color_optical_frame", "camera_link", ros::Time(0), ros::Duration(10));
    }
    catch (tf2::TransformException &ex) {
        ROS_WARN("%s",ex.what());
        ros::Duration(1.0).sleep();
    }
}

// void TRUE_RT_TAG::tf_static_cb(tf2_msgs::TFMessage msg)
// {
    
// }

bool TRUE_RT_TAG::loadTagConfig(std::string file_name)
{
    std::string file_path = package_path + "/config/" + file_name + ".yaml";

    ROS_INFO_STREAM("Loading tag configuration..." << file_path);

    YAML::Node node;
    try{
        node = YAML::LoadFile(file_path);
    }
    catch(std::exception &e)
    {
        ROS_ERROR("Failed to load tag config.");
        return false;

    }

    for(size_t tag = 0; tag < node.size(); tag++)
    {
        int32_t tag_rt_idx = node["TAG_TRUE_RT"]["TAGS"][tag][0].as<int32_t>();
        double tag_rt_size = node["TAG_TRUE_RT"]["TAGS"][tag][1].as<double>();

        tf2::Transform tag_rt;
        geometry_msgs::Point location;
        location.x = node["TAG_TRUE_RT"]["TAGS"][tag][2].as<double>();
        location.y = node["TAG_TRUE_RT"]["TAGS"][tag][3].as<double>();
        location.z = node["TAG_TRUE_RT"]["TAGS"][tag][4].as<double>();

        double heading = node["TAG_TRUE_RT"]["TAGS"][tag][5].as<double>();
        tf2::Quaternion orientation;
        orientation.setRotation(tf2::Vector3(0.0, 0.0, 1.0), DEG2RAD(heading));

        tag_rt.setOrigin(tf2::Vector3(location.x, location.y, location.z));
        tag_rt.setRotation(orientation);

        tag_rts._idxs.push_back(tag_rt_idx);
        tag_rts._sizes.push_back(tag_rt_size);
        tag_rts._transforms.push_back(tag_rt);
        // tag_rts._location.push_back(location);
        // tag_rts._heading.push_back(heading);

        ROS_INFO_STREAM("Tag info about " << tag_rt_idx << " located at " << tag_rt.getOrigin().getY() << " are loaded successfully");
    }

    return true;
}

void TRUE_RT_TAG::tag_detections_cb(apriltag_ros::AprilTagDetectionArray msg)
{
    if(msg.detections.size() > 0)
        tag_detection = msg;
    else
        ROS_WARN("There is no detected tags...");
}

bool TRUE_RT_TAG::getTrueRT()
{
    if(tag_detection.header.stamp < ros::Time::now() - ros::Duration(1.0))
    {
        ROS_ERROR("Detection data are out-dated!!!");
        return false;
    }

    if(!initialized)
    {
        ROS_ERROR("Configuration is not initialized...");
        return false;
    }

    // Selecting minimum distance tag to compute ground truth pose
    std::vector<double> dist_to_tag;
    for(int32_t idx = 0; idx < tag_detection.detections.size(); idx++)
        dist_to_tag.push_back(sqrt(
            pow(tag_detection.detections.at(idx).pose.pose.pose.position.x, 2) + 
            pow(tag_detection.detections.at(idx).pose.pose.pose.position.y, 2) +
            pow(tag_detection.detections.at(idx).pose.pose.pose.position.z, 2)));
    
    std::vector<double>::iterator min = std::min_element(dist_to_tag.begin(), dist_to_tag.end());

    int32_t min_idx = std::distance(dist_to_tag.begin(), min);
    double min_dist = *min;

    int32_t id = tag_detection.detections.at(min_idx).id.at(0);

    ROS_INFO_STREAM("Tag " << id << " at dist" << min_dist);

    // Find a ground truth RT wrt {WORLD} of identified tag
    int config_idx = - 99;
    config_idx = std::distance(tag_rts._idxs.begin(), std::find(tag_rts._idxs.begin(), tag_rts._idxs.end(), id));
    ROS_INFO_STREAM("Tag config found at " << config_idx);

    tf2::Transform global2tag_tf = tag_rts._transforms.at(config_idx);

    // Get transformation of camera to closest tag
    geometry_msgs::Pose cam2tag_pose = tag_detection.detections.at(min_idx).pose.pose.pose;
    tf2::Transform cam2tag_tf;
    tf2::fromMsg(cam2tag_pose, cam2tag_tf);

    tf2::fromMsg(cam2optical_geo.transform, cam2optical_tf);

    tf2::Transform global2_cam_tf = global2tag_tf*cam2tag_tf.inverse();

    geometry_msgs::Pose global2cam_pose;
    tf2::toMsg(global2_cam_tf, global2cam_pose);

    ROS_INFO_STREAM("Global pose of camera: " << global2cam_pose);

    return true;
}