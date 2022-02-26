#include <stdio.h>
#include <stdlib.h>

#include <ros/ros.h>
#include <ros/types.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <mmc_msgs/object_array_msg.h>
#include <mmc_msgs/object_msg.h>
#include <algorithm>     // sort 헤더파일
#include <vector>



using namespace std;

// pub클래스 만들기~~~

class radar_to_lidar_association{
  public:
    ros::Publisher pub0;
    radar_to_lidar_association();

    void callback_front_radar(const mmc_msgs::object_array_msg& data);
    void callback_front_lidar(const mmc_msgs::object_array_msg& data);
};

radar_to_lidar_association::radar_to_lidar_association(){
  cout << "-Association Node- Initializing..." << endl;
  
}

vector<float>radar_coordinate;  //할당 받아서 저장
int num_radar_coordinate;
// vector<pair<float,float>> radar_coordinate;
void radar_to_lidar_association::callback_front_radar(const mmc_msgs::object_array_msg& radar){

  radar_coordinate.clear(); // push_back을 하기 전에 초기화 하는 과정이 필요하다.

  for(short i=0; i!=radar.data.size(); i++){
    // cout<<"x = "<<radar.data[i].x<<endl;
    // cout<<"y = "<<radar.data[i].y<<endl;

    

    radar_coordinate.push_back(radar.data[i].x);
    radar_coordinate.push_back(radar.data[i].y);

    
  }
}
// list<float> list1;



void radar_to_lidar_association::callback_front_lidar(const mmc_msgs::object_array_msg& lidar){
    visualization_msgs::MarkerArray distance_vec;
    visualization_msgs::Marker line_strip;
    line_strip.header.frame_id = "sensors";
    line_strip.lifetime = ros::Duration(0.1);
    // line_strip.id = i;
    // line_strip.header.stamp = ros::Time::now();

    line_strip.type = visualization_msgs::Marker::LINE_STRIP;
    line_strip.ns = "radar_to_lidar_association";
    line_strip.action = visualization_msgs::Marker::ADD;
  
    num_radar_coordinate = radar_coordinate.size()/2;
    float arr_coordinate[num_radar_coordinate][lidar.data.size()];
    // float temp[num_radar_coordinate*lidar.data.size()];

    int distance_threshold, distance_minimum_threshold;
    distance_threshold = 5;
    distance_minimum_threshold = 0.1;
    float distance;

    /////////////////// 거리 //////////////////////
  
    for(short i=0; i!=num_radar_coordinate; i++){
    
        // line_strip.id = i;  
        int good = -1;
        for (short j=0; j!=lidar.data.size(); j++ ){   //행렬 sorting 하려면 s++ 해줘야함
            
            distance = sqrt(pow(radar_coordinate[2*i] - lidar.data[j].x, 2)+pow(radar_coordinate[2*i+1] - lidar.data[j].y , 2));
            arr_coordinate[i][j] = distance;

        }
    }

    /////////////////// radar기준 거리 //////////////////////

    for(short i=0; i!=num_radar_coordinate; i++){
        int good = -1; // good 연결될 lidar를 뜻함
        float save_value = distance_threshold + 0.1;
        for (short j=0; j!=lidar.data.size(); j++){
            if(arr_coordinate[i][j] > distance_threshold || arr_coordinate[i][j] > save_value){
                arr_coordinate[i][j]=0;
            }
            else{
                if(good>=0){
                    arr_coordinate[i][good] = 0;
                }
                save_value = arr_coordinate[i][j];
                good = j;    
            }
        // cout << "arr_coordinate["<<i<<"]["<<j<<"] =" << arr_coordinate[i][j] << endl;
        }
        // cout << i << "th radar :"  << good << endl;
    }
      //   if(good!= -1){
      //   cout << "[Actual lidar point info]" << endl;
      //   cout << good << "'s info : " << "x : " << lidar.data[good].x << "  y : " << lidar.data[good].y << endl;
      // }
        


    ///////////////////// lidar기준 거리 ////////////////////
    for(short j=0; j!=lidar.data.size(); j++){
      int good_r = -1; // good_r 연결된 radar 뜻함
      float save_r_value = distance_threshold + 0.1;
      for (short i=0; i!=num_radar_coordinate; i++){
        if(arr_coordinate[i][j] < save_r_value && arr_coordinate[i][j]!= 0) {
          if(good_r>=0){
            save_r_value = arr_coordinate[good_r][j];
          }
          save_r_value = arr_coordinate[i][j];
          good_r = i;
        }
      }

    
      if(good_r != -1){
        // cout << "We are in the good place " << endl;
        //line_strip.id = i;  
        line_strip.id = good_r;  

        //linestrip의 크기(너비) 및 색깔 
        line_strip.scale.x = 0.5f;  
        line_strip.scale.y = 0.2f;
        // line_strip.scale.x = 1.5f;
        line_strip.color.r = 0.1f;
        line_strip.color.g = 0.1f;
        line_strip.color.b = 0.5f;
        line_strip.color.a = 1.0f;  
        
        // Create the vertices for the points and lines
        // 라이다의 위치좌표
        geometry_msgs::Point good_point;
        good_point.x = lidar.data[j].x;
        good_point.y = lidar.data[j].y;


        // good_point.x = lidar.data[good_r].x;
        // good_point.y = lidar.data[good_r].y;
        good_point.z = 0;
        
        // line_strip.points.push_back(good_point);
        // cout<<"1"<<endl;
        geometry_msgs::Point good_point_r;
        good_point_r.x = radar_coordinate[2*good_r];
       // good_point_r.x = radar_coordinate[2*i];

        // cout<<"2"<<endl;
        good_point_r.y = radar_coordinate[2*good_r+1];
        //good_point_r.y = radar_coordinate[2*i+1];
        // cout<<"3"<<endl;
        line_strip.points.push_back(good_point);
        line_strip.points.push_back(good_point_r);

        pub0.publish(line_strip);
        // cout << line_strip << endl;
        distance_vec.markers.push_back(line_strip);
        // cout << distance_vec << endl;
        // cout << "[Publishing lidar point info]" << endl;
        // cout << good_point << endl;
        line_strip.points.clear();
      }

      }
    

    pub0.publish(distance_vec);
    distance_vec.markers.clear();    
    //}
          

}


int main(int argc, char **argv){
  ros::init(argc, argv, "radar_to_lidar_association");
  ros::NodeHandle node;


  radar_to_lidar_association rtla;

  ros::AsyncSpinner spinner(4);
  spinner.start();

  ros::Subscriber sub1  = node.subscribe("/sensors/front_radar", 1,         &radar_to_lidar_association::callback_front_radar, &rtla);
  ros::Subscriber sub2  = node.subscribe("/sensors/front_lidar", 1,         &radar_to_lidar_association::callback_front_lidar, &rtla);
  
  // ros publish 하기
  rtla.pub0 = node.advertise<visualization_msgs::MarkerArray>("/rviz/sensors/front_radar_association",1);

  ros::waitForShutdown();
}


//  ##################################################### Re #########################################################

// #include <stdio.h>
// #include <stdlib.h>

// #include <ros/ros.h>
// #include <ros/types.h>
// #include <visualization_msgs/Marker.h>
// #include <visualization_msgs/MarkerArray.h>

// #include <mmc_msgs/object_array_msg.h>
// #include <mmc_msgs/object_msg.h>
// #include <algorithm>     // sort 헤더파일
// #include <vector>
// #include <time.h>
// #include <sensor_msgs/PointCloud.h>
// #include "Eigen/Core"
// #include "Eigen/Dense"


// using namespace std;

// class track_initiation{

//   public:
//   track_initiation();

//   ros::Publisher pub0;

//   void callback_front_radar(const mmc_msgs::object_array_msg& data);
//   void callback_front_lidar(const mmc_msgs::object_array_msg& data);
//   void callback_tracking(const sensor_msgs::PointCloud& point_cloud);


//   vector <float> time0, time1, time2, time3, time4;
//   vector <float> lidar_data;
//   short time_step = 0;
// };

// track_initiation::track_initiation(){
//   cout << "-Association Node- Initializing..." << endl;
// }
// vector<float>radar_coordinate;

// void track_initiation::callback_front_radar(const mmc_msgs::object_array_msg& radar){
//   radar_coordinate.clear();

//   for(short i=0; i!=radar.data.size(); i++){

//     radar_coordinate.push_back(radar.data[i].x);
//     radar_coordinate.push_back(radar.data[i].y);    
//   }

// }


// void track_initiation::callback_front_lidar(const mmc_msgs::object_array_msg& data){       /// pointcloud ///

//   time_t start, end;
//   float result;
//   start = clock();

//   short period_time = 1;
//   if ( time_step < period_time){
//     time_step = time_step+1;
//   }

  
//  // 현재 시점 k step과 이전 k-1에서의 시점만 가지고 우선 생성해보기. time1, time2만    asociating된 애들 +1점씩 넣어주기! 변수 score같은거 선언해주기
//   auto get_time1 = time2;
//   auto get_time0 = time1;

//   time1 = get_time1;
//   time0 = get_time0;

//   time2.clear();
//   for (short i=0; i<data.data.size(); i++){
//     time2.push_back(data.data[i].x);
//     time2.push_back(data.data[i].y);
//     // cout << "x좌표 " << time4[i] << "y좌표" << time4[i] << endl;
//   }

//   float distance_threshold = 5;

//   // t초와 t-1초 비교
//   Eigen::MatrixXf arr2_1(time2.size()/2, time1.size()/2);
//   arr2_1.setZero();
//   for (short i=0; i<time2.size()/2; i++){
//     float distance;
//     for(short j=0; j<=time1.size()/2; j++){
//       distance = sqrt(pow(time2[i]-time1[j],2)+pow(time2[i+1]-time1[j+1],2));
//       arr2_1(i,j) = distance;
//       }
//   }


//   ///////////////////// radar  기준거리 //////////////////////
//   for (short i=0; i<time2.size()/2; i++){
//     float save_value = distance_threshold+0.5;
//     short good = -1; //연결될 lidar
//     for (short j=0; j<time1.size()/2; j++){
//       if (arr2_1(i,j) > distance_threshold || arr2_1(i, j)>= save_value){
//         arr2_1(i, j) = 0;
//       }
//       else{
//         if (good >=0){
//           arr2_1(i, good) = 0;
//         }
//         save_value = arr2_1(i, j);
//         good = j;
//       }
//     }
//   }

//     ///////////////////// lidar  기준거리 //////////////////////
    
//     for(short j=0; j!=time1.size()/2; j++){
//       int good_r = -1; // good_r 연결된 radar 뜻함
//       float save_r_value = distance_threshold + 0.1;
//       for (short i=0; i!=time2.size()/2; i++){
//         if(arr2_1(i, j) < save_r_value && arr2_1(i, j)!= 0) {
//           if(good_r>=0){
//             save_r_value = arr2_1(good_r,j);
//           }
//           save_r_value = arr2_1(i,j);
//           good_r = i;
//         }
//       }
//     }






//     ////////////////// 이거 time step 단계별로 update하는 logic짜고  몇번 반복하는 과정 밑에 짜기....    그럼 tracking 끝? //////////////////








// }

// int main(int argc, char **argv){
//   ros::init(argc, argv, "track_initiation");
//   ros::NodeHandle node;


//   track_initiation track_init;

//   ros::AsyncSpinner spinner(4);
//   spinner.start();

//   ros::Subscriber sub1  = node.subscribe("/sensors/front_radar", 1,         &track_initiation::callback_front_radar, &track_init);
//   ros::Subscriber sub2  = node.subscribe("/sensors/front_lidar", 1,         &track_initiation::callback_front_lidar, &track_init);
  
//   // ros publish 하기
//   track_init.pub0 = node.advertise<visualization_msgs::MarkerArray>("/rviz/sensors/track_initiation",1);

//   ros::waitForShutdown();
// }




//////////////////////////////////////// Starts from here ////////////////////////////////////////

// #include <stdio.h>
// #include <stdlib.h>

// #include <ros/ros.h>
// #include <ros/types.h>
// #include <visualization_msgs/Marker.h>
// #include <visualization_msgs/MarkerArray.h>

// #include <mmc_msgs/object_array_msg.h>
// #include <mmc_msgs/object_msg.h>
// #include <algorithm>     // sort 헤더파일
// #include <vector>
// #include <time.h>
// #include <sensor_msgs/PointCloud.h>
// #include "Eigen/Core"
// #include "Eigen/Dense"


// using namespace std;

// // pub클래스 만들기~~~


// class track_initiation{

//   public:
//   track_initiation();

//   ros::Publisher pub0;

//   void callback_front_radar(const mmc_msgs::object_array_msg& data);
//   void callback_front_lidar(const mmc_msgs::object_array_msg& data);
//   void callback_tracking(const sensor_msgs::PointCloud& point_cloud);

//   float distance_row_save_value;
//   float distance_column_save_value;
//   float distance_threshold;
//   vector <float> time0, time1, time2;
//   vector <float> lidar_data;
//   short time_step = 0;

// };

// track_initiation::track_initiation(){
//   cout << "-Association Node- Initializing..." << endl;
  
// };




// vector<float>radar_coordinate;  //할당 받아서 저장
// int num_radar_coordinate;
// // vector<pair<float,float>> radar_coordinate;
// void track_initiation::callback_front_radar(const mmc_msgs::object_array_msg& radar){

//   auto get_time1 = time2;
//   auto get_time0 = time1;

//   time1 = get_time1;
//   time0 = get_time0;


//   radar_coordinate.clear(); // push_back을 하기 전에 초기화 하는 과정이 필요하다.

//   for(short i=0; i!=radar.data.size(); i++){
//     // cout<<"x = "<<radar.data[i].x<<endl;
//     // cout<<"y = "<<radar.data[i].y<<endl;
//     time2.clear();
//     time2.push_back(radar.data[i].x);
//     time2.push_back(radar.data[i].y);
//   }

//   distance_threshold = 2.0;
//   vector <short> smae_score(time2.size()/2,0);
//   float distance;


//   //////////////////////// distance calculation ////////////////////////

//   Eigen::MatrixXf two_to_one(time2.size()/2,time1.size()/2);
//   two_to_one.setZero();
//   for(short i=0; i<time2.size(); i++){
//     for(short j=0; j<time1.size(); j++){
//       distance = sqrt(pow(time2[2*i]-time1[2*j],2) + pow(time2[2*i+1]-time1[2*j+1],2));
//       two_to_one(i,j) = distance;
//     }

//   }

//   /////////////////// radar 기준 거리 (row) ////////////////////////
//   for(short i=0; i<time2.size(); i++){
//     int good = -1;
//     distance_row_save_value = distance_threshold + 0.1;
//     for(short j=0; j<time1.size(); j++){
//     if (distance > distance_threshold || distance > distance_row_save_value){
//       two_to_one(i,j) = 0;
//     }
//     else{
//       if(good>=0){
//         two_to_one(i,good) = 0;
//       }
//       distance_row_save_value = two_to_one(i,j);
//       good = j;
//       }
//     }
//   }


//   /////////////////// radar 기준 거리 (column) ////////////////////////

//   for(short j=0; j<time1.size(); j++){
//     int good_column = -1;
//     distance_column_save_value = distance_threshold + 0.1;
//     for(short i=0; i<time2.size(); i++){
//       if(two_to_one(i,j)< distance_column_save_value && two_to_one(i,j) != 0){
//         if(good_column >=0){
//           distance_column_save_value = two_to_one(good_column,j);
//         }
//         distance_column_save_value = two_to_one(i,j);
//         good_column = i;
//       }
//       // cout << two_to_one(i,j) << endl;
//     }
//   }


  













// }

// // list<float> list1;







// int main(int argc, char **argv){
//   ros::init(argc, argv, "track_initiation");
//   ros::NodeHandle node;


//   track_initiation track_init;

//   ros::AsyncSpinner spinner(4);
//   spinner.start();

//   ros::Subscriber sub1  = node.subscribe("/sensors/front_radar", 1,         &track_initiation::callback_front_radar, &track_init);
  
//   // ros publish 하기
//   track_init.pub0 = node.advertise<visualization_msgs::MarkerArray>("/rviz/sensors/front_radar_association",1);

//   ros::waitForShutdown();
// }






















// int main(int argc, char **argv){
//   ros::init(argc, argv, "radar_to_lidar_association");
//   ros::NodeHandle node;


//   radar_to_lidar_association rtla;

//   ros::AsyncSpinner spinner(4);
//   spinner.start();

//   ros::Subscriber sub1  = node.subscribe("/sensors/front_radar", 1,         &radar_to_lidar_association::callback_front_radar, &rtla);
//   ros::Subscriber sub2  = node.subscribe("/sensors/front_lidar", 1,         &radar_to_lidar_association::callback_front_lidar, &rtla);
  
//   // ros publish 하기
//   rtla.pub0 = node.advertise<visualization_msgs::MarkerArray>("/rviz/sensors/front_radar_association",1);

//   ros::waitForShutdown();
// }


