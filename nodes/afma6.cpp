#include <stdio.h>
#include <math.h>
#include <sstream>

#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/JointState.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

#include <visp/vpRobotAfma6.h> // visp

#include <visp_bridge/3dpose.h> // visp_bridge


#ifdef VISP_HAVE_AFMA6

class RosAfma6Node
{
  public:
    RosAfma6Node(ros::NodeHandle n);
    virtual ~RosAfma6Node();
    
  public:
    int setup();
    void setCameraVel( const geometry_msgs::TwistStampedConstPtr &);
    void setJointPos( const sensor_msgs::JointStateConstPtr &);
    void setJointVel( const sensor_msgs::JointStateConstPtr &);
    void spin();
    void publish();
 
  protected:
    ros::NodeHandle n;
    ros::Publisher pose_pub;
    ros::Publisher ee_pose_pub;
    ros::Publisher vel_pub;
    ros::Publisher joints_pub;
    ros::Subscriber cmd_camvel_sub;
    ros::Subscriber cmd_jointpos_sub;
    ros::Subscriber cmd_jointvel_sub;

    ros::Time cmdTime;

    std::string serial_port;

    vpRobotAfma6 *robot;
    boost::mutex robotMutex;

    geometry_msgs::PoseStamped position;
    geometry_msgs::PoseStamped ee_position;
    		
    //for world->ee_link transform
    tf::TransformBroadcaster tf_broadcaster;
//    geometry_msgs::TransformStamped ee_trans;

    //for resolving tf names.
    std::string tf_prefix;
    std::string frame_id_odom;
    std::string frame_id_base_link;

    vpHomogeneousMatrix wMc; // world to camera transformation
    vpHomogeneousMatrix wMe; // world to ee transformation
    vpColVector q, qDot; // measured joint position and velocity
 };


RosAfma6Node::RosAfma6Node(ros::NodeHandle nh)
{
    // read in config options
    n = nh;

    ROS_INFO( "using Afma6 robot" );

    robot = NULL;
    /*
     * Figure out what frame_id's to use. if a tf_prefix param is specified,
     * it will be added to the beginning of the frame_ids.
     *
     * e.g. rosrun ... _tf_prefix:=MyRobot (or equivalently using <param>s in
     * roslaunch files)
     * will result in the frame_ids being set to /MyRobot/odom etc,
     * rather than /odom. This is useful for Multi Robot Systems.
     * See ROS Wiki for further details.
     */
    //  tf_prefix = tf::getPrefixParam(n);
    //  frame_id_odom = tf::resolve(tf_prefix, "odom");
    //  frame_id_base_link = tf::resolve(tf_prefix, "base_link");

    // advertise services
    pose_pub = n.advertise<geometry_msgs::PoseStamped>("pose", 1000);
    ee_pose_pub = n.advertise<geometry_msgs::PoseStamped>("ee_pose", 1000);
    vel_pub = n.advertise<geometry_msgs::TwistStamped>("velocity", 1000);
    joints_pub = n.advertise<sensor_msgs::JointState>("joints", 1000);

    // subscribe to services
    cmd_camvel_sub = n.subscribe( "cmd_camvel", 1, (boost::function < void(const geometry_msgs::TwistStampedConstPtr&)>) boost::bind( &RosAfma6Node::setCameraVel, this, _1 ));
    cmd_jointpos_sub = n.subscribe<sensor_msgs::JointState>( "cmd_jointpos", 1, &RosAfma6Node::setJointPos, this);
    cmd_jointvel_sub = n.subscribe<sensor_msgs::JointState>( "cmd_jointvel", 1, &RosAfma6Node::setJointVel, this);
}

RosAfma6Node::~RosAfma6Node()
{
    if (robot) {
        robot->stopMotion();
        delete robot;
        robot = NULL;
    }
}

int RosAfma6Node::setup()
{
    robot = new vpRobotAfma6;

    robot->init(vpAfma6::TOOL_CCMOP, vpCameraParameters::perspectiveProjWithDistortion);

    if (!robot->getPowerState()){
    	ROS_WARN("Robot is OFF. Turn it on now.");
    	robot->powerOn();
    }

    try{
    	robot->setRobotState(vpRobot::STATE_STOP);
    } catch (vpException& e) {
    	ROS_ERROR("In RosAfma6Node::setup()%s", e.what());
    }


    return 0;
}

void RosAfma6Node::spin()
{
    ros::Rate loop_rate(200);


    while(ros::ok()){
        ros::Time now = ros::Time::now();
        const double elapsed = (now - cmdTime).toSec();    
        if (robot->getRobotState()==vpRobot::STATE_VELOCITY_CONTROL && elapsed>(1./30.)){
            ROS_WARN("It's been a long time since we received a command.");
            if (elapsed>(1./1.)){
            	ROS_ERROR("It's been a very long time since we received a command.");
                try{
                	robot->setRobotState(vpRobot::STATE_STOP);
                } catch (vpException& e) {
                	ROS_ERROR("In RosAfma6Node::spin():%s", e.what());
                }
            }
        }
        this->publish();
        ros::spinOnce();
        loop_rate.sleep();
    }
    //  ros::spin();
}

void RosAfma6Node::publish()
{
    double timestamp;

    boost::mutex::scoped_lock scopedRobotMutex(robotMutex);
    try {
    	robot->getPosition(vpRobot::ARTICULAR_FRAME, q, timestamp);
    	wMc = robot->get_fMc(q);
    	robot->get_fMe(q, wMe);
    } catch (vpException& e){
    	ROS_ERROR("In RosAfma6Node::publish():%s", e.what());
    	try{
			robot->setRobotState(vpRobot::STATE_STOP);
		} catch (vpException& e) {
			ROS_ERROR("In RosAfma6Node::publish():%s", e.what());
		}
//    	robot->setRobotState(vpRobot::STATE_STOP);
//    	ros::shutdown();
    }
    position.pose = visp_bridge::toGeometryMsgsPose(wMc);
    position.header.stamp = ros::Time(timestamp); // to improve: should be the timestamp returned by getPosition()

    //  ROS_INFO( "Afma6 publish pose at %f s: [%0.2f %0.2f %0.2f] - [%0.2f %0.2f %0.2f %0.2f]",
    //            position.header.stamp.toSec(),
    //            position.pose.position.x, position.pose.position.y, position.pose.position.z,
    //            position.pose.orientation.w, position.pose.orientation.x, position.pose.orientation.y, position.pose.orientation.z);
    pose_pub.publish(position);
    tf::Stamped<tf::Pose> tfPosition;
    tf::poseStampedMsgToTF(position, tfPosition);
    tf_broadcaster.sendTransform(tf::StampedTransform(tfPosition, ros::Time::now(), "base", "Dragonfly2-8mm-ccmop"));

    ee_position.pose = visp_bridge::toGeometryMsgsPose(wMe);
    ee_position.header.stamp = position.header.stamp;
    ee_pose_pub.publish(ee_position);
    tf::Stamped<tf::Pose> tfEe;
	tf::poseStampedMsgToTF(ee_position, tfEe);
	tf_broadcaster.sendTransform(tf::StampedTransform(tfEe, ros::Time::now(), "base", "end_effector"));


    vpColVector vel(6);
    try {
    	robot->getVelocity(vpRobot::CAMERA_FRAME, vel, timestamp);
    } catch (vpException& e){
    	ROS_ERROR("In RosAfma6Node::publish():%s", e.what());
    	try{
			robot->setRobotState(vpRobot::STATE_STOP);
		} catch (vpException& e) {
			ROS_ERROR("In RosAfma6Node::publish():%s", e.what());
		}
//    	ros::shutdown();
	}
    geometry_msgs::TwistStamped vel_msg;
    vel_msg.header.stamp = ros::Time(timestamp);
    vel_msg.twist.linear.x = vel[0];
    vel_msg.twist.linear.y = vel[1];
    vel_msg.twist.linear.z = vel[2];
    vel_msg.twist.angular.x = vel[3];
    vel_msg.twist.angular.y = vel[4];
    vel_msg.twist.angular.z = vel[5];
    vel_pub.publish(vel_msg);


    try{
    	robot->getVelocity(vpRobot::ARTICULAR_FRAME, qDot, timestamp);
    } catch (vpException& e){
    	ROS_ERROR("In RosAfma6Node::publish():%s", e.what());
		try{
			robot->setRobotState(vpRobot::STATE_STOP);
		} catch (vpException& e) {
			ROS_ERROR("In RosAfma6Node::publish():%s", e.what());
		}
	}
    sensor_msgs::JointState joint_msg;
    joint_msg.header.stamp = ros::Time(timestamp);
    joint_msg.position.resize(6);
    memcpy(joint_msg.position.data(), q.data, 6*sizeof(double));
    joint_msg.velocity.resize(6);
    memcpy(joint_msg.velocity.data(), qDot.data, 6*sizeof(double));
    joints_pub.publish(joint_msg);


    //	ros::Duration(1e-3).sleep();
}

void RosAfma6Node::setCameraVel( const geometry_msgs::TwistStampedConstPtr &msg)
{
    cmdTime = ros::Time::now();

    vpColVector vc(6); // Vel in m/s and rad/s

    vc[0] = msg->twist.linear.x;
    vc[1] = msg->twist.linear.y;
    vc[2] = msg->twist.linear.z;

    vc[3] = msg->twist.angular.x;
    vc[4] = msg->twist.angular.y;
    vc[5] = msg->twist.angular.z;

    //  ROS_INFO( "Afma6 new camera vel at %f s: [%0.2f %0.2f %0.2f] m/s [%0.2f %0.2f %0.2f] rad/s",
    //            cmdTime.toSec(),
    //            vc[0], vc[1], vc[2], vc[3], vc[4], vc[5]);
    boost::mutex::scoped_lock scopedRobotMutex(robotMutex);

    try {
    	robot->setRobotState(vpRobot::STATE_VELOCITY_CONTROL);
    	robot->setVelocity(vpRobot::CAMERA_FRAME, vc);
    } catch (vpException& e) {
    	ROS_ERROR("%s", e.what());
    }

}


void RosAfma6Node::setJointPos( const sensor_msgs::JointStateConstPtr &msg)
{
    if (msg->position.size() == 6){
        cmdTime = ros::Time::now();

        vpColVector j(6); // Vel in m/s and rad/s

        memcpy(j.data, msg->position.data(), 6*sizeof(double));

        ROS_INFO( "Afma6 new joint pos at %f s: [%0.2f %0.2f %0.2f] m/s [%0.2f %0.2f %0.2f] rad/s",
                cmdTime.toSec(),
                j[0], j[1], j[2], j[3], j[4], j[5]);
        boost::mutex::scoped_lock scopedRobotMutex(robotMutex);

        try {
        	robot->setRobotState(vpRobot::STATE_POSITION_CONTROL);
			robot->setPosition(vpRobot::ARTICULAR_FRAME, j);
		} catch (vpException& e) {
			ROS_ERROR("In RosAfma6Node::setJointPos( const sensor_msgs::JointStateConstPtr &msg):%s", e.what());
		}
	}
}

void RosAfma6Node::setJointVel( const sensor_msgs::JointStateConstPtr &msg)
{
    if (msg->velocity.size() == 6){
        cmdTime = ros::Time::now();

        vpColVector jVel(6); // Vel in m/s and rad/s

        memcpy(jVel.data, msg->velocity.data(), 6*sizeof(double));

        ROS_INFO( "Afma6 new joint vel at %f s: [%0.2f %0.2f %0.2f] m/s [%0.2f %0.2f %0.2f] rad/s",
                cmdTime.toSec(),
                jVel[0], jVel[1], jVel[2], jVel[3], jVel[4], jVel[5]);
        boost::mutex::scoped_lock scopedRobotMutex(robotMutex);
        try {
			robot->setRobotState(vpRobot::STATE_VELOCITY_CONTROL);
        	robot->setVelocity(vpRobot::ARTICULAR_FRAME, jVel);
		} catch (vpException& e) {
			ROS_ERROR("In RosAfma6Node::setJointVel( const sensor_msgs::JointStateConstPtr &msg):%s", e.what());
		}
    }
}

#endif // #ifdef VISP_HAVE_AFMA6

int main( int argc, char** argv )
{
#ifdef VISP_HAVE_AFMA6
  ros::init(argc,argv, "RosAfma6");
  ros::NodeHandle n(std::string("~"));

  RosAfma6Node *node = new RosAfma6Node(n);

  if( node->setup() != 0 )
  {
    printf( "Afma6 setup failed... \n" );
    return -1;
  }

  node->spin();

  delete node;

  printf( "\nQuitting... \n" );
#else
  printf("This node is not available since ViSP was \nnot built with Afma6 robot support...\n");
#endif
  return 0;
}

