#include "ros/ros.h"  // rosで必要はヘッダーファイル
#include <geometry_msgs/Twist.h> // ロボットを動かすために必要
using namespace std;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pyramid_ugv_teleop");
  // initでROSを初期化し、my_teleopという名前をノードにつける
  // 同じ名前のノードが複数あるとだめなので、ユニークな名前をつける

  ros::NodeHandle nh;
  // ノードハンドラの作成。ハンドラは必要時に起動される。

  ros::Publisher  pub;
  // パブリッシャの作成。トピックに対してデータを送信。

  ros::Rate rate(10);
  // ループの頻度を設定。この場合は10Hz、1秒間に10回数、1ループ100ms。

  geometry_msgs::Twist vel;
  // geometry_msgs::Twist　この型は並進速度と回転速度(vector3:3次元ベクトル) を合わせたもので、速度指令によく使われる

  pub= nh.advertise<geometry_msgs::Twist>("/husky_velocity_controller/cmd_vel", 10);
  // マスターにgeometry_msgs::Twist型のデータを送ることを伝える
  // マスターは/cmd_vel_mux/input/teleopトピック(1番目の引数）を購読する
  // 全てのノードにトピックができたことを知らせる(advertise)。
  // 2番目の引数はデータのバッファサイズ

  std::cout << "f: forward, b: backward, r: right, l:left" << std::endl;

while (ros::ok()) // このノードが使える間は無限ループ
    {
      char key;  // 入力キーの値

      cin >> key;
      cout << key << endl;

      switch (key) {
      case 'f': vel.linear.x  =  0.5; break;
      case 'b': vel.linear.x  = -0.5; break;
      case 'l': vel.angular.z =  1.0; break;
      case 'r': vel.angular.z = -1.0; break;
        // linear.xは前後方向の並進速度(m/s)
        // angular.zは回転速度(rad/s)
      }

      pub.publish(vel); // 速度メッセージを送信
      ros::spinOnce();  // コールバック関数を呼ぶ
      vel.linear.x  = 0.0; // 並進速度の初期化
      vel.angular.z = 0.0; // 回転速度の初期化
      rate.sleep();     // 指定した周期でループするよう寝て待つ


    }

  return 0;
}
