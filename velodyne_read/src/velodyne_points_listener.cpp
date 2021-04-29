/**
 * 该例程将订阅/velodyne_points话题，消息类型sensor_msgs::PointCloud2
 */
 
#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>

using namespace std;

// 定义重要参数
const float FRONT_BOUNDARY = 0.5;
const float BACK_BOUNDARY = -0.5;
const float LEFT_BOUNDARY = 0.5;
const float RIGHT_BOUNDARY = -0.5;
const float UP_BOUNDARY = 0.5;
const float DOWN_BOUNDARY = -0.5;

const float NEGATIVE_EDGE = -0.2;
const float POSITIVE_EDGE = 0.2;
const float ZERO_EDGE = 0;

// 回调函数
void chatterCallback(const sensor_msgs::PointCloud2 &msg)
{
    // 格式转化，为了显示
	sensor_msgs::PointCloud out_pcd;
	sensor_msgs::convertPointCloud2ToPointCloud(msg, out_pcd);

    // 打印点云的XYZ坐标
	for (int i=0; i<out_pcd.points.size(); i++)
    {
        float y = out_pcd.points[i].x;
        float x = out_pcd.points[i].y;
        float z = out_pcd.points[i].z;

        // 打印所有点坐标
		// cout << x << "," << y << "," << z << "\n";
        
        // 障碍物检测，暂时忽略z方向
        if(z > ZERO_EDGE && z < POSITIVE_EDGE)
        {
            if(x > NEGATIVE_EDGE && x < POSITIVE_EDGE)
            {
                if(y > ZERO_EDGE && y < FRONT_BOUNDARY)
                    cout << y << ",front\n"; // 前方警告
                else if(y < ZERO_EDGE && y > BACK_BOUNDARY)
                    cout<< y << ",back\n"; // 后方警告
            }
            if(y > NEGATIVE_EDGE && y < POSITIVE_EDGE)
            {
                if(x > ZERO_EDGE && x < LEFT_BOUNDARY)
                    cout << x << ",left\n"; // 左侧警告
                else if(x < ZERO_EDGE && x > RIGHT_BOUNDARY)
                    cout << x << ",right\n"; // 右侧警告
            }            
        }  
	}
}

int main(int argc, char **argv)
{
    cout<<"---------Start--------\n";
    // 初始化ROS节点
    ros::init(argc, argv, "listener");
    // 创建节点句柄
    ros::NodeHandle n;
    // 创建Subscriber
    ros::Subscriber points_sub = n.subscribe("/velodyne_points", 10, chatterCallback);
    // 循环等待回调函数
    ros::spin();
    cout<<"---------End--------\n";
    return 0;
}