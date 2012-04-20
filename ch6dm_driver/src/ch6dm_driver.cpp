#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <boost/asio.hpp>
#include <boost/array.hpp>
#include <tf/transform_broadcaster.h>
#include <std_msgs/Float32.h>

#define DEG2RAD 0.017453293
#define SCALE_FACTOR 0.0109863

void rotate(double heading, double attitude, double bank, geometry_msgs::Quaternion * quat)
{
    // Assuming the angles are in radians.
    double c1 = cos(heading / 2);
    double s1 = sin(heading / 2);
    double c2 = cos(attitude / 2);

    double s2 = sin(attitude / 2);
    double c3 = cos(bank / 2);
    double s3 = sin(bank / 2);
    double c1c2 = c1 * c2;
    double s1s2 = s1 * s2;

    quat->w = c1c2 * c3 - s1s2 * s3;
    quat->x = c1c2 * s3 + s1s2 * c3;
    quat->y = s1 * c2 * c3 + c1 * s2 * s3;
    quat->z = c1 * s2 * c3 - s1 * c2 * s3;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "ch6dm_publisher");
    ros::NodeHandle n;
    ros::NodeHandle priv_nh("~");

    std::string port;
    int baud_rate = 115200;
    std::string frame_id1;
    std::string frame_id2;

    priv_nh.param("port", port, std::string("/dev/ttyUSB0"));
    //priv_nh.param("baud_rate", baud_rate, 115200);
    priv_nh.param("baud_rate", baud_rate, 115200);
    priv_nh.param("frame_id1", frame_id1, std::string("odom"));
    priv_nh.param("frame_id2", frame_id2, std::string("imu"));

    boost::asio::io_service io;
    boost::array < uint8_t, 16 > raw_bytes_;
    boost::array < uint8_t, 2 > roll_;
    boost::array < uint8_t, 2 > pitch_;
    boost::array < uint8_t, 2 > yaw_;
    boost::array < uint8_t, 2 > sum_;

    ros::Rate loop_rate(200);

    try {
        boost::asio::serial_port serial_(io, port);
        serial_.set_option(boost::asio::serial_port_base::baud_rate(baud_rate));

        ros::Publisher imu_pub = n.advertise < sensor_msgs::Imu > ("imu", 50);
        sensor_msgs::Imu imu;

        imu.header.frame_id = frame_id2;
        imu.header.stamp = ros::Time::now();

        rotate(0.0, 0.0, 0.0, &(imu.orientation));

        //imu.orientation.x = 0;//tf::createQuaternionFromRPY(0.0, 0.0, 0.0);

        tf::TransformBroadcaster broadcaster;


        uint8_t init_level = 0;
        uint8_t index;
        uint8_t temp_char;

        double roll = 0.0;
        double pitch = 0.0;
        double yaw = 0.0;

        while (ros::ok()) {

            ros::spinOnce();
            //scan.header.frame_id = frame_id;

            if (init_level == 0) {
                boost::asio::read(serial_, boost::asio::buffer(&temp_char, 1));
                // start byte
                if (temp_char == 's')
                    init_level = 1;
                else
                    init_level = 0;
            }
            if (init_level == 1) {
                // position index 
                boost::asio::read(serial_, boost::asio::buffer(&temp_char, 1));
//                ROS_ERROR("step1");

                if (temp_char == 'n') {
                    init_level++;
                } else {
                    init_level = 0;
                }
            }
            if (init_level == 2) {
                // position index 
                boost::asio::read(serial_, boost::asio::buffer(&temp_char, 1));
//                ROS_ERROR("step2");

                if (temp_char == 'p') {
                    init_level++;
                } else {
                    init_level = 0;
                }
            }
            if (init_level == 3) {
                // position index 
                boost::asio::read(serial_, boost::asio::buffer(&temp_char, 1));
//                ROS_ERROR("step3");

                if (temp_char == 0xB7) {
                    init_level++;
                } else {
                    init_level = 0;
                }
            }
            if (init_level == 4) {


                boost::asio::read(serial_, boost::asio::buffer(&temp_char, 1));
                boost::asio::read(serial_, boost::asio::buffer(&temp_char, 1));
                boost::asio::read(serial_, boost::asio::buffer(&temp_char, 1));
                //boost::asio::read(serial_, boost::asio::buffer(&roll_, 2));

                boost::asio::read(serial_, boost::asio::buffer(&yaw_, 2));
                yaw = -(((double)( (int16_t)((int16_t)yaw_[0] << 8) + (int16_t) yaw_[1]) ) * SCALE_FACTOR) * DEG2RAD;

                ROS_ERROR("yaw %f", yaw);

                boost::asio::read(serial_, boost::asio::buffer(&pitch_, 2));
                pitch = (((double)( (int16_t)((int16_t)pitch_[0] << 8) + (int16_t) pitch_[1]) ) * SCALE_FACTOR) * DEG2RAD;

                ROS_ERROR("pitch %f", pitch);

                // roll
                boost::asio::read(serial_, boost::asio::buffer(&roll_, 2));
                roll = -(((double)( (int16_t)((int16_t)roll_[0] << 8) + (int16_t) roll_[1]) ) * SCALE_FACTOR) * DEG2RAD;
                //ROS_ERROR("Speed 1 : %d", (((speed_[1]<<8) + speed_[0]) >> 6));
                //ROS_ERROR("Speed 2 : %d", ((((speed_[1]<<8) + speed_[0]) >> 6) / 60));
                //ROS_ERROR("Speed 3 : %f", scan.time_increment);

                ROS_ERROR("roll %f", roll);



                boost::asio::read(serial_, boost::asio::buffer(&sum_, 2));

                //imu.orientation = tf::createQuaternionFromRPY(roll, pitch, yaw);
                rotate(roll, yaw, pitch, &(imu.orientation));

                imu_pub.publish(imu);

                init_level = 0;
                broadcaster.sendTransform(tf::StampedTransform(tf::Transform(tf::createQuaternionFromRPY(roll, pitch, yaw), tf::Vector3(0.0, 0.0, 0.1)),
                                                           ros::Time::now(), frame_id1, frame_id2));
            }
  //   loop_rate.sleep();

        }


    }
    catch(boost::system::system_error ex) {
        ROS_ERROR("Error instantiating IMU object. Are you sure you have the correct port and baud rate? Error was %s", ex.what());
        return -1;
    }
}

