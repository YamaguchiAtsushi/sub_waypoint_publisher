#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <map>
#include <cmath>
#include <vector>
#include <std_msgs/Empty.h>
#include <time.h>
#include <fstream>
#include <sstream>
#include <string>

struct Person
{
    std::vector<geometry_msgs::Point> points;  // 検出された点の集まり
    double average_distance;                   // 平均距離
    double length;                             // バウンディングボックスの長さ
    double aspect_ratio;                       // アスペクト比
    double timeout_counter;
    geometry_msgs::Point min_point;            // バウンディングボックスの最小点
    geometry_msgs::Point max_point;            // バウンディングボックスの最大点
    int id;                                    // 人を一意に識別するID
    bool is_lost;
    bool is_tracked;                           // この人が追跡されているかどうかのフラグ
};

class LidarClustering
{
public:
    LidarClustering() : next_id_(0)
    {
        scan_sub_ = nh_.subscribe("/scan", 10, &LidarClustering::scanCallback, this);
        odom_sub_ = nh_.subscribe("ypspur_ros/odom", 10, &LidarClustering::odomCallback, this);
        amcl_sub_ = nh_.subscribe("/amcl_pose", 1000, &LidarClustering::amclPoseCallback, this);

        sub_waypoint_state_sub_ = nh_.subscribe("/sub_waypoint_state", 10, &LidarClustering::subWaypointStateCallback, this);
        waypoint_sub_ = nh_.subscribe("waypoint", 1000, &LidarClustering::waypointCallback, this);


        cluster_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("lidar_clusters", 10);
        people_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("detected_people", 10);
        sub_waypoint_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("sub_waypoint", 10);
        points_pub_ = nh_.advertise<visualization_msgs::Marker>("points_marker", 10); // ポイント用パブリッシャー

        robot_r_.x = 0.0;
        robot_r_.y = 0.0;
        robot_r_.z = 0.0;
        robot_r_.w = 1.0;
    }


    void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
    {
        // クラスター用のベクトルを初期化
        std::vector<std::vector<geometry_msgs::Point>> clusters;
        std::vector<geometry_msgs::Point> current_cluster;

        // 各スキャンごとに初期化すべき変数を宣言
        double distance_threshold = 0.1;
        double cluster_sum_x = 0.0;
        double cluster_sum_y = 0.0;
        double cluster_distance_min = std::numeric_limits<double>::max();
        int cluster_distance_min_number = -1; // 最も近いクラスターの番号
        double point_x = 0.0;
        double point_y_min = std::numeric_limits<double>::max();
        double point_y_max = std::numeric_limits<double>::lowest();
        int cluster_flag = 0;


        // double waypoint_angle_rad = std::atan2(goal_.pose.position.y - robot_odom_y_ , goal_.pose.position.x - robot_odom_x_); // ラジアン
        double waypoint_angle_rad = std::atan2(goal_.pose.position.y - robot_y_ , goal_.pose.position.x - robot_x_); // ラジアン


        // スキャンデータの各レンジについてクラスタリング処理
        for (size_t i = 0; i < scan->ranges.size(); ++i)
        {
            if (std::isfinite(scan->ranges[i]))
            {
                geometry_msgs::Point p;
                double angle = scan->angle_min + i * scan->angle_increment;
                p.x = scan->ranges[i] * cos(angle);
                p.y = scan->ranges[i] * sin(angle);

                // 現在のクラスタに点を追加
                if (current_cluster.empty() || distance(current_cluster.back(), p) < distance_threshold)
                {
                    current_cluster.push_back(p);
                }
                else
                {
                    if (!current_cluster.empty())
                    {
                        clusters.push_back(current_cluster); // 完了したクラスタを保存
                        current_cluster.clear(); // 新しいクラスタのためにクリア
                    }
                    current_cluster.push_back(p);
                }
            }
        }

        // 残ったクラスタを追加
        if (!current_cluster.empty())
        {
            clusters.push_back(current_cluster);
        }

        // クラスタごとの処理
        for (size_t i = 0; i < clusters.size(); ++i)
        {
            // std::cout << "cluster_size: " << clusters[i].size() << std::endl;
            float distance = scan->ranges[i];

            cluster_sum_x = 0.0; // 各クラスタごとに初期化
            cluster_sum_y = 0.0;
            cluster_flag = 0;

            //各cluster内の点に関して、短冊内にあるかどうか
            for (const auto& point : clusters[i])
            {
                // double angle_rad = std::atan2(point.y, point.x) - waypoint_angle_rad; // ラジアン //次のwaypointを加味できてる？？？
                double angle_rad = std::atan2(point.y, point.x);

                // ROS_INFO("point.x:%f", point.x);
                // ROS_INFO("point.y:%f", point.y);
                // std::cout << "waytoint_angle_rad(度数法):" << waypoint_angle_rad * (180.0 / M_PI) << std::endl;
                // std::cout << "angle_rad(度数法):" << angle_rad * (180.0 / M_PI) << std::endl;

                double angle_deg = angle_rad * (180.0 / M_PI);   // 度に変換

                if (-10 < angle_deg && angle_deg < 10) 
                {
                    // float distance = scan->ranges[i];
                    double distance_judge = distance * std::cos(angle_rad);

                    if (-1.5 < distance_judge && distance_judge < 1.5) 
                    {
                        // cluster_sum_x += point.x;
                        // cluster_sum_y += point.y;
                        cluster_flag = 1;
                        // std::cout << "cluster_flag" << cluster_flag << std::endl;
                    }
                }


                //横を検知しすぎるので消した・あとで復活を考える
                // else if((angle_deg >= -90 && angle_deg <= -10) || (angle_deg >= 10 && angle_deg <= 90)){
                //       double distance_judge = distance * std::sin(angle_rad);
                //       if(-0.5 < distance_judge && distance_judge < 0.5){
                //         cluster_flag = 1;
                //       }
                // }

                cluster_sum_x += point.x;
                cluster_sum_y += point.y;
            }

            if (cluster_flag == 1) // フラグが立っている場合
            {
                // std::cout << "cluster_flag on!!!" << std::endl;
                double cluster_ave_x = cluster_sum_x / clusters[i].size();
                double cluster_ave_y = cluster_sum_y / clusters[i].size();
                double cluster_distance = sqrt(pow(cluster_ave_x, 2) + pow(cluster_ave_y, 2));

                if (cluster_distance_min > cluster_distance && 0.1 <cluster_distance && cluster_distance < 1.5) // 1.5m以内のクラスターの中で最も近いクラスターを探す // 0.1m以上のクラスターを探す(ロボット周辺のノイズを除去)
                {
                    cluster_distance_min = cluster_distance;
                    cluster_distance_min_number = i;
                }
                cluster_flag = 0;
            }
        }


        // 最も近いクラスターについて、最小・最大のy座標、および平均x座標を計算
        if (cluster_distance_min_number != -1)
        {
            double sum_x = 0.0; // クラスタ内のx座標の合計
            int point_count = 0; // クラスタ内の点の数

            std::cout << "cluster_distance_min_number: " << cluster_distance_min_number << std::endl;

            for (const auto& point : clusters[cluster_distance_min_number])
            {
                // x座標の合計を計算
                sum_x += point.x;
                ++point_count;

                // y座標の最小・最大値を更新
                if (point.y < point_y_min)
                {
                    point_y_min = point.y;
                    // std::cout << "point_y_min: " << point_y_min << std::endl;
                }
                if (point.y > point_y_max)
                {
                    point_y_max = point.y;
                    // std::cout << "point_y_max: " << point_y_max << std::endl;
                }
            }

            // 平均x座標を計算
            if (point_count > 0)
            {
                point_x = sum_x / point_count; // 各点の平均x座標
                std::cout << "point_x (average): " << point_x << std::endl;
                std::cout << "point_y_min: " << point_y_min << std::endl;
                std::cout << "point_y_max: " << point_y_max << std::endl;

            }

            // デバッグ用出力
            // std::cout << "point_x (average): " << point_x 
            //         << ", point_y_min: " << point_y_min 
            //         << ", point_y_max: " << point_y_max << std::endl;
        }


        // // 最も近いクラスターについて、最小・最大のy座標を計算
        // if (cluster_distance_min_number != -1)
        // {
        //     for (const auto& point : clusters[cluster_distance_min_number])
        //     {
        //         point_x = point.x;//各点の平均をとったほうがいいかもーーーーーーーーーーー

        //         if (point.y < point_y_min)
        //         {
        //             point_y_min = point.y;
        //         }
        //         if (point.y > point_y_max)
        //         {
        //             point_y_max = point.y;
        //         }
        //     }

        //     // std::cout << "point_y_min: " << point_y_min << " point_y_max: " << point_y_max << std::endl;
        // }

        // if(abs(point_y_min) < abs(point_y_max)){
        //     sub_waypoint_msg.pose.position.x =point_x;
        //     sub_waypoint_msg.pose.position.y =point_y_min - 0.3; //0.3m左にずらす
        // }
        // else{
        //     sub_waypoint_msg.pose.position.x =point_x;
        //     sub_waypoint_msg.pose.position.y =point_y_max + 0.3; //0.3m右にずらす
        // }
        if(abs(point_y_min) < abs(point_y_max)){

            if(point_x != 0.0){
                // sub_waypoint_msg.pose.position.x = robot_odom_x_ + point_x + 0.3;
                // sub_waypoint_msg.pose.position.y = robot_odom_y_ + point_y_min - 0.5; //0.3m右にずらす
                sub_waypoint_msg.pose.position.x = robot_x_ + point_x + 0.1;
                sub_waypoint_msg.pose.position.y = robot_y_ + point_y_min - 0.5; //0.3m右にずらす
                std::cout << "IN SUB WAYPOINT_1 point_x (average): " << point_x << std::endl;
                std::cout << "IN SUB WAYPOINT_1 point_y_min: " << point_y_min << std::endl;
                std::cout << "IN SUB WAYPOINT_1 point_y_max: " << point_y_max << std::endl;
            }
        }
        else{
            if(point_x != 0.0){//loop_rateがscanの更新周期なので、他のノードに初期化された値を渡すのを防ぐ
                // sub_waypoint_msg.pose.position.x = robot_odom_x_ + point_x + 0.3;
                // sub_waypoint_msg.pose.position.y = robot_odom_y_ + point_y_max + 0.5; //0.3m左にずらす
                sub_waypoint_msg.pose.position.x = robot_x_ + point_x + 0.1;
                sub_waypoint_msg.pose.position.y = robot_y_ + point_y_max + 0.5; //0.3m左にずらす
                std::cout << "IN SUB WAYPOINT_2 point_x (average): " << point_x << std::endl;
                std::cout << "IN SUB WAYPOINT_2 point_y_min: " << point_y_min << std::endl;
                std::cout << "IN SUB WAYPOINT_2 point_y_max: " << point_y_max << std::endl;
            }
        }

        // std::cout << "sub_waypoint_state: " << sub_waypoint_state_ << std::endl;
        if(sub_waypoint_state_ == 1){
            sub_waypoint_pub_.publish(sub_waypoint_msg);
            // ROS_INFO("-----------PUBLISH_WAYPOINT----------------");

            // std::cout << "robot_odom_x:" << robot_odom_x_ << "robot_odom_y:" << robot_odom_y_ << std::endl;
            // std::cout << "point_x: " << point_x << " point_y_min: " << point_y_min << " point_y_max: " << point_y_max << std::endl;
            // std::cout << "sub_waypoint_x: " << sub_waypoint_msg.pose.position.x << " sub_waypoint_y: " << sub_waypoint_msg.pose.position.y << std::endl;

        std::vector<visualization_msgs::Marker> point_markers; // ポイント用マーカーのリスト

        // マーカーの設定
        visualization_msgs::Marker points_marker;
        points_marker.header.frame_id = "laser"; // 適切な座標フレーム名に変更
        points_marker.header.stamp = ros::Time::now();
        points_marker.ns = "points";
        points_marker.id = 0;
        points_marker.type = visualization_msgs::Marker::POINTS;
        points_marker.action = visualization_msgs::Marker::ADD;
        points_marker.pose.orientation.w = 1.0;
        points_marker.scale.x = 0.3; // 点のサイズ
        points_marker.scale.y = 0.3;
        points_marker.color.a = 1.0;  // 透明度
        points_marker.color.r = 1.0;  // 色 (赤)
        points_marker.color.g = 0.0;
        points_marker.color.b = 0.0;

        points_marker.points.push_back(sub_waypoint_msg.pose.position); // ウェイポイントを追加
        points_marker.points.push_back(sub_waypoint_msg.pose.position); // ウェイポイントを追加


            // ROS_INFO("-----------PUBLISH_WAYPOINT----------------", sub_waypoint_msg.pose.position.x);
            // ROS_INFO("-----------PUBLISH_WAYPOINT----------------", sub_waypoint_msg.pose.position.y);
        }

        // std::cout << "point_x: " << point_x << " point_y_min: " << point_y_min << " point_y_max: " << point_y_max << std::endl;
    }

    void waypointCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        // ROS_INFO("Received waypoint: [x: %f, y: %f]", msg->pose.position.x, msg->pose.position.y);

        // サブスクライブしたウェイポイントを処理
        // 例えば次の目標に設定する
        goal_ = *msg;
        // std::cout << "goal_" << goal_ << std::endl;
    }

    void odomCallback(const nav_msgs::Odometry::ConstPtr& odom)
    {
        robot_odom_x_ = odom->pose.pose.position.x;
        robot_odom_y_ = odom->pose.pose.position.y;
        // std::cout << "robot_odom_x:" << robot_odom_x_ << "robot_odom_y:" << robot_odom_y_ << std::endl;
    }

    void amclPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) {
        robot_x_ = msg->pose.pose.position.x;
        robot_y_ = msg->pose.pose.position.y;
        robot_r_ = msg->pose.pose.orientation;
        // ROS_INFO("Current estimated pose: [robot_x_: %f, robot_y_: %f, theta: %f]", robot_x_, robot_y_, tf::getYaw(msg->pose.pose.orientation));
    }

    void subWaypointStateCallback(const std_msgs::Int16::ConstPtr& sub_waypoint_state)
    {
        sub_waypoint_state_ = sub_waypoint_state->data;
    }


private:
    ros::NodeHandle nh_;
    ros::Subscriber scan_sub_, odom_sub_, sub_waypoint_state_sub_, amcl_sub_, waypoint_sub_;
    ros::Publisher cluster_pub_;
    ros::Publisher people_pub_;
    ros::Publisher sub_waypoint_pub_;
    ros::Publisher points_pub_;

    std::vector<Person> tracked_people_; // 追跡中の人々のリスト
    std::vector<int> deleted_ids; //削除される人のID
    geometry_msgs::PoseStamped sub_waypoint_msg;
    geometry_msgs::PoseStamped goal_;  // 目標地点goal_judge 
    geometry_msgs::Quaternion robot_r_;

    int next_id_; // 次に使用するID
    double cluster_distance_min = 100000;
    int cluster_distance_min_number;
    int cluster_flag = 0;
    double cluster_sum_x = 0.0;
    double cluster_sum_y = 0.0;
    double point_y_min;
    double point_y_max;

    double robot_odom_x_, robot_odom_y_;
    double robot_x_, robot_y_;
    int sub_waypoint_state_ = 0;

    

    double distance(const geometry_msgs::Point& p1, const geometry_msgs::Point& p2)
    {
        return sqrt(pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2));
    }

    void publishClusters(const std::vector<std::vector<geometry_msgs::Point>>& clusters)
    {
        visualization_msgs::MarkerArray marker_array;
        for (size_t i = 0; i < clusters.size(); ++i)
        {
            visualization_msgs::Marker marker;
            marker.header.frame_id = "laser";
            marker.header.stamp = ros::Time::now();
            marker.ns = "clusters";
            marker.id = i;
            marker.type = visualization_msgs::Marker::LINE_STRIP;
            marker.action = visualization_msgs::Marker::ADD;
            marker.scale.x = 0.05;
            marker.color.r = 0.0;
            marker.color.g = 1.0;
            marker.color.b = 0.0;
            marker.color.a = 1.0;

            for (const auto& point : clusters[i])
            {
                marker.points.push_back(point);
            }

            marker_array.markers.push_back(marker);
            //addTextMarker(marker_array, i, marker.points[0]);
        }

        cluster_pub_.publish(marker_array);
    }
    



};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "sub_waypoint_publisher");
    LidarClustering lc;
    ros::spin();
    return 0;
}
