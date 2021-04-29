#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sstream>
#include <sensor_msgs/LaserScan.h>
#include <velodyne_msgs/VelodyneScan.h>
#include <velodyne_msgs/VelodynePacket.h>
#include <sensor_msgs/point_cloud2_iterator.h>

// 回调函数
void chatterCallback(const velodyne_msgs::VelodyneScan::ConstPtr& msg)
{
    //ROS_INFO("I heard: [%X]", msg->packets[0].data[0]);
    //ROS_INFO("I heard: [%X]", msg->packets[0].data[1]);//test
	//计算水平角
    float angle1=256 * msg->packets[3].data[3] + msg->packets[3].data[2];
    float angle2=angle1 / 100;//calculate angle
	//计算十六线的激光打出的点距离，注，16个激光头的垂直角是固定的
	
	float rela_distance[16] = {0};
	
    float distance1=256 * msg->packets[3].data[5] + msg->packets[3].data[4];
    rela_distance[0]=distance1 * 2 / 1000;//calculate  angle-15 distance
    
    float distance3=256 * msg->packets[3].data[8] + msg->packets[3].data[7];
    rela_distance[1]=distance3 * 2 / 1000;//calculate  angle1 distance

    float distance5=256 * msg->packets[3].data[11] + msg->packets[3].data[10];
    rela_distance[2]=distance5 * 2 / 1000;//calculate  angle-13 distance

    float distance7=256 * msg->packets[3].data[14] + msg->packets[3].data[13];
    rela_distance[3]=distance7 * 2 / 1000;//calculate  angle-3 distance

    float distance9=256 * msg->packets[3].data[17] + msg->packets[3].data[16];
    rela_distance[4]=distance9 * 2 / 1000;//calculate  angle-11 distance

    float distance11=256 * msg->packets[3].data[20] + msg->packets[3].data[19];
    rela_distance[5]=distance11 * 2 / 1000;//calculate  angle5 distance

    float distance13=256 * msg->packets[3].data[23] + msg->packets[3].data[22];
    rela_distance[6]=distance13 * 2 / 1000;//calculate  angle-9 distance

    float distance15=256 * msg->packets[3].data[26] + msg->packets[3].data[25];
    rela_distance[7]=distance15 * 2 / 1000;//calculate  angle7 distance

    float distance17=256 * msg->packets[3].data[29] + msg->packets[3].data[28];
    rela_distance[8]=distance17 * 2 / 1000;//calculate  angle-7 distance
    
    float distance19=256 * msg->packets[3].data[32] + msg->packets[3].data[31];
    rela_distance[9]=distance19 * 2 / 1000;//calculate  angle9 distance

    float distance21=256 * msg->packets[3].data[35] + msg->packets[3].data[34];
    rela_distance[10]=distance21 * 2 / 1000;//calculate  angle-5 distance

    float distance23=256 * msg->packets[3].data[38] + msg->packets[3].data[37];
    rela_distance[11]=distance23 * 2 / 1000;//calculate  angle11 distance

    float distance25=256 * msg->packets[3].data[41] + msg->packets[3].data[40];
    rela_distance[12]=distance25 * 2 / 1000;//calculate  angle-3 distance

    float distance27=256 * msg->packets[3].data[44] + msg->packets[3].data[43];
    rela_distance[13]=distance27 * 2 / 1000;//calculate  angle13 distance

    float distance29=256 * msg->packets[3].data[47] + msg->packets[3].data[46];
    rela_distance[14]=distance29 * 2 / 1000;//calculate  angle-1 distance

    float distance31=256 * msg->packets[3].data[50] + msg->packets[3].data[49];
    rela_distance[15]=distance31 * 2 / 1000;//calculate  angle15 distance
	
	// 打印水平角信息和16个相对距离
    ROS_INFO("angle:%f\ndistance:\n%f,%f,%f,%f\n%f,%f,%f,%f\n%f,%f,%f,%f\n%f,%f,%f,%f\n", 
		angle2, rela_distance[0], rela_distance[1], rela_distance[2], rela_distance[3],
 		rela_distance[4],rela_distance[5],rela_distance[6],rela_distance[7],
		rela_distance[8],rela_distance[9],rela_distance[10],rela_distance[11],
		rela_distance[12],rela_distance[13],rela_distance[14],rela_distance[15]
	);
}
int main(int argc, char **argv)
{
    std::cout<<"--------Start--------\n";
    ros::init(argc, argv, "listener");
    ros::NodeHandle n;
    ros::Rate loop_rate(100);
	
    ros::Subscriber sub = n.subscribe<velodyne_msgs::VelodyneScan>("/velodyne_packets",100,chatterCallback);
    while(1)
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
    std::cout<<"--------End--------\n";
    return 0;
}