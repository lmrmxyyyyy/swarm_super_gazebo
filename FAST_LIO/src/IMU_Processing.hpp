#include <cmath>
#include <math.h>
#include <deque>
#include <mutex>
#include <thread>
#include <fstream>
#include <csignal>
#include <ros/ros.h>
#include <so3_math.h>
#include <Eigen/Eigen>
#include <common_lib.h>
#include <pcl/common/io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <condition_variable>
#include <nav_msgs/Odometry.h>
#include <pcl/common/transforms.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <tf/transform_broadcaster.h>
#include <eigen_conversions/eigen_msg.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Vector3.h>
#include "use-ikfom.hpp"

/// *************Preconfiguration

#define MAX_INI_COUNT (100)
//判断lidar点时间先后
const bool time_list(PointType &x, PointType &y) {return (x.curvature < y.curvature);};

/// *************IMU Process and undistortion
class ImuProcess
{
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  ImuProcess();
  ~ImuProcess();
  
  void Reset();
  void Reset(double start_timestamp, const sensor_msgs::ImuConstPtr &lastimu);
  void set_extrinsic(const V3D &transl, const M3D &rot);
  void set_extrinsic(const V3D &transl);
  void set_extrinsic(const MD(4,4) &T);
  void set_gyr_cov(const V3D &scaler);
  void set_acc_cov(const V3D &scaler);
  void set_gyr_bias_cov(const V3D &b_g);
  void set_acc_bias_cov(const V3D &b_a);
  Eigen::Matrix<double, 12, 12> Q;
  void Process(const MeasureGroup &meas,  esekfom::esekf<state_ikfom, 12, input_ikfom> &kf_state, PointCloudXYZI::Ptr pcl_un_);

  ofstream fout_imu;       // imu参数输出文件
  V3D cov_acc;             //加速度测量协方差
  V3D cov_gyr;             //角速度测量协方差
  V3D cov_acc_scale;       //加速度测量协方差
  V3D cov_gyr_scale;       //角速度测量协方差
  V3D cov_bias_gyr;        //角速度测量协方差偏置
  V3D cov_bias_acc;        //加速度测量协方差偏置
  double first_lidar_time; //当前帧第一个点云时间

 private:
  void IMU_init(const MeasureGroup &meas, esekfom::esekf<state_ikfom, 12, input_ikfom> &kf_state, int &N);
  void UndistortPcl(const MeasureGroup &meas, esekfom::esekf<state_ikfom, 12, input_ikfom> &kf_state, PointCloudXYZI &pcl_in_out);

  PointCloudXYZI::Ptr cur_pcl_un_;        //当前帧点云未去畸变
  sensor_msgs::ImuConstPtr last_imu_;     // 上一帧imu
  deque<sensor_msgs::ImuConstPtr> v_imu_; // imu队列
  vector<Pose6D> IMUpose;                 // imu位姿
  vector<M3D> v_rot_pcl_;                 //未使用
  M3D Lidar_R_wrt_IMU;                    // lidar到IMU的旋转外参
  V3D Lidar_T_wrt_IMU;                    // lidar到IMU的位置外参
  V3D mean_acc;                           //加速度均值,用于计算方差
  V3D mean_gyr;                           //角速度均值，用于计算方差
  V3D angvel_last;                        //上一帧角速度
  V3D acc_s_last;                         //上一帧加速度
  double start_timestamp_;                //开始时间戳
  double last_lidar_end_time_;            //上一帧结束时间戳
  int init_iter_num = 1;                  //初始化迭代次数
  bool b_first_frame_ = true;             //是否是第一帧
  bool imu_need_init_ = true;             //是否需要初始化imu
};

ImuProcess::ImuProcess()
    : b_first_frame_(true), imu_need_init_(true), start_timestamp_(-1)
{
  init_iter_num = 1;                          //初始化迭代次数
  Q = process_noise_cov();                    //调用use-ikfom.hpp里面的process_noise_cov完成噪声协方差的初始化
  cov_acc = V3D(0.1, 0.1, 0.1);               //加速度测量协方差初始化
  cov_gyr = V3D(0.1, 0.1, 0.1);               //角速度测量协方差初始化
  cov_bias_gyr = V3D(0.0001, 0.0001, 0.0001); //角速度测量协方差偏置初始化
  cov_bias_acc = V3D(0.0001, 0.0001, 0.0001); //加速度测量协方差偏置初始化
  mean_acc = V3D(0, 0, -1.0);
  mean_gyr = V3D(0, 0, 0);
  angvel_last = Zero3d;                    //上一帧角速度初始化
  Lidar_T_wrt_IMU = Zero3d;                // lidar到IMU的位置外参初始化
  Lidar_R_wrt_IMU = Eye3d;                 // lidar到IMU的旋转外参初始化
  last_imu_.reset(new sensor_msgs::Imu()); //上一帧imu初始化
}

ImuProcess::~ImuProcess() {}
//重置参数
void ImuProcess::Reset()
{
  // ROS_WARN("Reset ImuProcess");
  mean_acc = V3D(0, 0, -1.0);
  mean_gyr = V3D(0, 0, 0);
  angvel_last = Zero3d;
  imu_need_init_ = true;                   //是否需要初始化imu
  start_timestamp_ = -1;                   //开始时间戳
  init_iter_num = 1;                       //初始化迭代次数
  v_imu_.clear();                          // imu队列清空
  IMUpose.clear();                         // imu位姿清空
  last_imu_.reset(new sensor_msgs::Imu()); //上一帧imu初始化
  cur_pcl_un_.reset(new PointCloudXYZI()); //当前帧点云未去畸变初始化
}

//传入外参，包含R,T
void ImuProcess::set_extrinsic(const MD(4,4) &T)
{
  Lidar_T_wrt_IMU = T.block<3,1>(0,3);
  Lidar_R_wrt_IMU = T.block<3,3>(0,0);
}
//传入外参，包含T
void ImuProcess::set_extrinsic(const V3D &transl)
{
  Lidar_T_wrt_IMU = transl;
  Lidar_R_wrt_IMU.setIdentity();
}
//传入外参，包含R,T
void ImuProcess::set_extrinsic(const V3D &transl, const M3D &rot)
{
  Lidar_T_wrt_IMU = transl;
  Lidar_R_wrt_IMU = rot;
}
// 传入陀螺仪角速度协方差
void ImuProcess::set_gyr_cov(const V3D &scaler)
{
  cov_gyr_scale = scaler;
}
// 传入加速度计加速度协方差
void ImuProcess::set_acc_cov(const V3D &scaler)
{
  cov_acc_scale = scaler;
}
// 传入陀螺仪角速度协方差偏置
void ImuProcess::set_gyr_bias_cov(const V3D &b_g)
{
  cov_bias_gyr = b_g;
}
// 传入加速度计加速度协方差偏置
void ImuProcess::set_acc_bias_cov(const V3D &b_a)
{
  cov_bias_acc = b_a;
}

void ImuProcess::IMU_init(const MeasureGroup &meas, esekfom::esekf<state_ikfom, 12, input_ikfom> &kf_state, int &N)
{
  /** 1. initializing the gravity, gyro bias, acc and gyro covariance
   ** 2. normalize the acceleration measurenments to unit gravity **/
   /** 1.初始化重力，加速度、陀螺仪偏差协方差。
   ** 2. 将加速度归一化到单位重力 **/
  ROS_INFO("IMU Initializing: %.1f %%", double(N) / MAX_INI_COUNT * 100);
  V3D cur_acc, cur_gyr;
  
  if (b_first_frame_)//如果是第一帧数据
  {
    Reset(); //重置参数
    N = 1;   //将迭代次数置1
    b_first_frame_ = false;
    const auto &imu_acc = meas.imu.front()->linear_acceleration;//这里都是IMU数据//
    const auto &gyr_acc = meas.imu.front()->angular_velocity;
    mean_acc << imu_acc.x, imu_acc.y, imu_acc.z;
    mean_gyr << gyr_acc.x, gyr_acc.y, gyr_acc.z;//用于后面计算平均加速度和角速度
    first_lidar_time = meas.lidar_beg_time;//获得第一帧激光雷达开始的时间
  }
 //计算方差
  for (const auto &imu : meas.imu)
  {//遍历测量信息中存放的imu信息
    const auto &imu_acc = imu->linear_acceleration;
    const auto &gyr_acc = imu->angular_velocity;
    cur_acc << imu_acc.x, imu_acc.y, imu_acc.z;
    cur_gyr << gyr_acc.x, gyr_acc.y, gyr_acc.z;

    mean_acc      += (cur_acc - mean_acc) / N;
    mean_gyr      += (cur_gyr - mean_gyr) / N;
    //计算加速度和角速度的协方差值//套协方差的公式
    cov_acc = cov_acc * (N - 1.0) / N + (cur_acc - mean_acc).cwiseProduct(cur_acc - mean_acc) * (N - 1.0) / (N * N);
    cov_gyr = cov_gyr * (N - 1.0) / N + (cur_gyr - mean_gyr).cwiseProduct(cur_gyr - mean_gyr) * (N - 1.0) / (N * N);

    // cout<<"acc norm: "<<cur_acc.norm()<<" "<<mean_acc.norm()<<endl;

    N ++;
  }
  state_ikfom init_state = kf_state.get_x();//KF的初始状态

  //从common_lib.h中拿到重力，并与加速度测量均值的单位重力求出SO2的旋转矩阵类型的重力加速度
  init_state.grav = S2(- mean_acc / mean_acc.norm() * G_m_s2);
  
  //state_inout.rot = Eye3d; // Exp(mean_acc.cross(V3D(0, 0, -1 / scale_gravity)));
  init_state.bg  = mean_gyr;//角速度测量作为陀螺仪偏差
  init_state.offset_T_L_I = Lidar_T_wrt_IMU;//将lidar和imu外参位移量传入
  init_state.offset_R_L_I = Lidar_R_wrt_IMU;//将lidar和imu外参旋转量传入
  kf_state.change_x(init_state); //将初始化状态传入esekfom.hpp中的x_

//误差状态协方差初始值
  //在esekfom.hpp获得P_的协方差矩阵
  esekfom::esekf<state_ikfom, 12, input_ikfom>::cov init_P = kf_state.get_P();
  init_P.setIdentity();//初始化协方差矩阵为单位矩阵:
  init_P(6,6) = init_P(7,7) = init_P(8,8) = 0.00001; //将协方差矩阵的lidar和imu外参旋转量的协方差置为0.00001
  init_P(9,9) = init_P(10,10) = init_P(11,11) = 0.00001; //将协方差矩阵的lidar和imu外参位移量的协方差置为0.00001
  init_P(15,15) = init_P(16,16) = init_P(17,17) = 0.0001;//将协方差矩阵的陀螺仪偏差协方差置为0.0001
  init_P(18,18) = init_P(19,19) = init_P(20,20) = 0.001;//将协方差矩阵的加速度计偏差协方差置为0.001
  init_P(21,21) = init_P(22,22) = 0.00001; //重力的协方差置为0.0001
  kf_state.change_P(init_P);//将初始化协方差矩阵传入esekfom.hpp中的P_
  last_imu_ = meas.imu.back();//最后一个Imu数据

}

void ImuProcess::UndistortPcl(const MeasureGroup &meas, esekfom::esekf<state_ikfom, 12, input_ikfom> &kf_state, PointCloudXYZI &pcl_out)
{
  /*** add the imu of the last frame-tail to the of current frame-head ***/
  /*** 将上一帧结尾的IMU数据作为这一帧的开始，并获得当前帧激光雷达开始和结束的时间 ***/
  auto v_imu = meas.imu;
  v_imu.push_front(last_imu_); //将上一帧最后尾部的imu添加到当前帧头部的imu
  const double &imu_beg_time = v_imu.front()->header.stamp.toSec();
  const double &imu_end_time = v_imu.back()->header.stamp.toSec();
  const double &pcl_beg_time = meas.lidar_beg_time;
  const double &pcl_end_time = meas.lidar_end_time;
  
  /*** sort point clouds by offset time ***/
  /*** 通过时间先后点云进行排序 按之前已设定的曲率（单点时间戳）对点云进行排列***/
  pcl_out = *(meas.lidar);
  sort(pcl_out.points.begin(), pcl_out.points.end(), time_list);
  // cout<<"[ IMU Process ]: Process lidar from "<<pcl_beg_time<<" to "<<pcl_end_time<<", " \
  //          <<meas.imu.size()<<" imu msgs from "<<imu_beg_time<<" to "<<imu_end_time<<endl;

  /*** Initialize IMU pose ***/
  /***初始化IMU的位姿---设置好IMU位姿的数据格式***/
  state_ikfom imu_state = kf_state.get_x();
  IMUpose.clear();
  IMUpose.push_back(set_pose6d(0.0, acc_s_last, angvel_last, imu_state.vel, imu_state.pos, imu_state.rot.toRotationMatrix()));

  /*** forward propagation at each imu point ***/
  /*** 对每一个IMU数据进行前向传播***/
  // angvel_avr为平均角速度，acc_avr为平均加速度，acc_imu为imu加速度，vel_imu为imu速度，pos_imu为imu位置
  V3D angvel_avr, acc_avr, acc_imu, vel_imu, pos_imu;
  M3D R_imu;//imu到世界旋转矩阵

  double dt = 0;//时间增量

  input_ikfom in;//kf的输入
  // 遍历本次估计的所有IMU测量并且进行积分，离散中值法 前向传播
  for (auto it_imu = v_imu.begin(); it_imu < (v_imu.end() - 1); it_imu++)
  {//imu数据迭代
    auto &&head = *(it_imu);
    auto &&tail = *(it_imu + 1);
    //只选用在这一帧激光雷达时间里的imu数据
    if (tail->header.stamp.toSec() < last_lidar_end_time_)    continue;
    //获得当前帧下一帧两帧IMU数据的平均值
    angvel_avr<<0.5 * (head->angular_velocity.x + tail->angular_velocity.x),
                0.5 * (head->angular_velocity.y + tail->angular_velocity.y),
                0.5 * (head->angular_velocity.z + tail->angular_velocity.z);
    acc_avr   <<0.5 * (head->linear_acceleration.x + tail->linear_acceleration.x),
                0.5 * (head->linear_acceleration.y + tail->linear_acceleration.y),
                0.5 * (head->linear_acceleration.z + tail->linear_acceleration.z);

    // fout_imu << setw(10) << head->header.stamp.toSec() - first_lidar_time << " " << angvel_avr.transpose() << " " << acc_avr.transpose() << endl;
    //加速度去除重力影响////加速度平均值*重力加速度9.8/归一化后的加速度平均值
    acc_avr     = acc_avr * G_m_s2 / mean_acc.norm(); // - state_inout.ba;

    if(head->header.stamp.toSec() < last_lidar_end_time_)
    {//帧头的时间还在上一帧激光雷达时间区域中
      dt = tail->header.stamp.toSec() - last_lidar_end_time_;
      //获得帧尾到上一帧激光雷达结束时间的时间差
      // dt = tail->header.stamp.toSec() - pcl_beg_time;
    }
    else
    {//否则就IMU帧尾到帧头的时间差
      dt = tail->header.stamp.toSec() - head->header.stamp.toSec();
    }
    
    in.acc = acc_avr;
    in.gyro = angvel_avr;//获得输入的 加速度平均值，角速度平均值
     // 配置协方差矩阵
    //获得角速度，加速度以及相应偏差值的协方差
    Q.block<3, 3>(0, 0).diagonal() = cov_gyr;
    Q.block<3, 3>(3, 3).diagonal() = cov_acc;
    Q.block<3, 3>(6, 6).diagonal() = cov_bias_gyr;
    Q.block<3, 3>(9, 9).diagonal() = cov_bias_acc;
    //每收到一次imu数据帧则进行一次前向更新（先验）
    kf_state.predict(dt, Q, in);//获得一个预测的状态值//前向传播后的状态

    /* save the poses at each IMU measurements */
    /*保存IMU观测值的位姿 */
    imu_state = kf_state.get_x();
    angvel_last = angvel_avr - imu_state.bg;
    acc_s_last  = imu_state.rot * (acc_avr - imu_state.ba);//世界坐标系加速度
    for(int i=0; i<3; i++)
    {
      acc_s_last[i] += imu_state.grav[i];//加上重力得到真世界坐标系的加速度
    }
    //后一个IMU时刻距离此次雷达开始的时间间隔
    double &&offs_t = tail->header.stamp.toSec() - pcl_beg_time;
    //保存IMU预测过程的状态
    IMUpose.push_back(set_pose6d(offs_t, acc_s_last, angvel_last, imu_state.vel, imu_state.pos, imu_state.rot.toRotationMatrix()));
  }

  /*** calculated the pos and attitude prediction at the frame-end ***/
  /*** 计算帧尾位姿和姿态的预测值 ***/
  double note = pcl_end_time > imu_end_time ? 1.0 : -1.0;
  dt = note * (pcl_end_time - imu_end_time);
  //确保lidar与imu的处理时间对齐,在时间差上有区别
  //这里的时间差计算的是点云帧的结尾与IMU最后数据的时间
  //一帧激光雷达，中间的预测值时间差用IMU自己的，结束时预测值时间差用IMU和激光雷达的//
  kf_state.predict(dt, Q, in);
  
  imu_state = kf_state.get_x();//此时的状态是一帧结束时刻的
  last_imu_ = meas.imu.back();
  last_lidar_end_time_ = pcl_end_time;//记录IMU和激光雷达结束时刻的时间



  /*** undistort each lidar point (backward propagation) ***/
  /*** 对每一个激光雷达进行去畸变处理(反向传播) ***/
  // 在点云去畸变时，需求得lidar周期的多段离散时间完毕后的状态预测，将点云按该状态量转换到imu坐标系下处理，再转换回lidar坐标系。
  if (pcl_out.points.begin() == pcl_out.points.end()) return;
  auto it_pcl = pcl_out.points.end() - 1;
  for (auto it_kp = IMUpose.end() - 1; it_kp != IMUpose.begin(); it_kp--)
  {//从当前帧激光雷达中IMU数据从后往前迭代
    auto head = it_kp - 1;//从倒数第二帧IMU开始
    auto tail = it_kp;
    R_imu<<MAT_FROM_ARRAY(head->rot);//旋转阵
    // cout<<"head imu acc: "<<acc_imu.transpose()<<endl;
    vel_imu<<VEC_FROM_ARRAY(head->vel);//速度
    pos_imu<<VEC_FROM_ARRAY(head->pos);//位姿
    acc_imu<<VEC_FROM_ARRAY(tail->acc);//加速度
    angvel_avr<<VEC_FROM_ARRAY(tail->gyr);//角速度//加速度、角速度使用下一帧的数据
    //点云时间需要迟于前一个IMU时刻 因为是在两个IMU时刻之间去畸变，此时默认雷达的时间戳在后一个IMU时刻之前
    for(; it_pcl->curvature / double(1000) > head->offset_time; it_pcl --)
    {//这里的曲率是时间，除以1000单位化为s与IMU的偏移时间比较
      dt = it_pcl->curvature / double(1000) - head->offset_time;//点到IMU开始时刻的时间间隔

      /* Transform to the 'end' frame, using only the rotation
       * Note: Compensation direction is INVERSE of Frame's moving direction
       * So if we want to compensate a point at timestamp-i to the frame-e
       * P_compensate = R_imu_e ^ T * (R_i * P_i + T_ei) where T_ei is represented in global frame */
      /*变换到“结束”帧，仅使用旋转 *注意：补偿方向与帧的移动方向相反 *所以如果我们想补偿时间戳i到帧e的一个点 
       * P_compensate = R_imu_e ^ T * (R_i * P_i + T_ei) 其中T_ei在全局框架中表示*/
      //按时间戳的差值进行插值

      M3D R_i(R_imu * Exp(angvel_avr, dt));//点所在时刻的旋转
      V3D P_i(it_pcl->x, it_pcl->y, it_pcl->z);//点所在时刻的位置(雷达坐标系下)
      /* T_ei展开是pos_imu + vel_imu * dt + 0.5 * acc_imu * dt * dt - imu_state.pos也就是点所在时刻IMU在世界坐标系下的位置
       - end时刻IMU在世界坐标系下的位置 W^t_I-W^t_I_e*/
      V3D T_ei(pos_imu + vel_imu * dt + 0.5 * acc_imu * dt * dt - imu_state.pos);
      
      //雷达点世界坐标系偏移量
      //lidar点通过外参(Rp+t)转换到imu坐标系，在imu坐标系下向前一帧做反向变换，再转换回到lidar坐标系 
      // imu_state.offset_R_L_I是从雷达到惯性的旋转矩阵 简单记为I^R_L
      // imu_state.offset_T_L_I是惯性系下雷达坐标系原点的位置简单记为I^t_L
      // P_compensate是点在末尾时刻在雷达系的坐标 简记为L^P_e
      // imu_state.rot.conjugate()是结束时刻IMU到世界坐标系的旋转矩阵的转置 也就是(W^R_i_e)^T
      V3D P_compensate = imu_state.offset_R_L_I.conjugate() * (imu_state.rot.conjugate() * (R_i * (imu_state.offset_R_L_I * P_i + imu_state.offset_T_L_I) + T_ei) - imu_state.offset_T_L_I);// not accurate!
      //去畸变后的位姿
      
      // save Undistorted points and their rotation
      // 保存去畸变的点和旋转
      it_pcl->x = P_compensate(0);
      it_pcl->y = P_compensate(1);
      it_pcl->z = P_compensate(2);

      if (it_pcl == pcl_out.points.begin()) break;
    }
  }
}

// 对IMU数据进行预处理（初始化），其中包含了点云畸变处理 前向传播 反向传播
void ImuProcess::Process(const MeasureGroup &meas,  esekfom::esekf<state_ikfom, 12, input_ikfom> &kf_state, PointCloudXYZI::Ptr cur_pcl_un_)
{
  double t1,t2,t3;
  t1 = omp_get_wtime();

  if(meas.imu.empty()) {return;};// 拿到的当前帧的imu测量为空，则直接返回
  ROS_ASSERT(meas.lidar != nullptr);

  if (imu_need_init_)
  { /// 第一个激光雷达帧
    /// The very first lidar frame
    IMU_init(meas, kf_state, init_iter_num);//这里进行IMU的初始化工作**

    imu_need_init_ = true;
    
    last_imu_   = meas.imu.back();

    state_ikfom imu_state = kf_state.get_x();//将KF初始化后得到的状态给IMU状态
    if (init_iter_num > MAX_INI_COUNT)//如果迭代帧数多于20便不需要初始化
    {
      // if (init_iter_num == MAX_INI_COUNT+1)
      // {
      //   Eigen::Quaterniond rotation = Eigen::Quaterniond::FromTwoVectors(mean_acc, Eigen::Vector3d::UnitZ());
      //   mean_acc = rotation * mean_acc;
      //   mean_gyr = rotation * mean_gyr;
      //   state_ikfom init_state = kf_state.get_x();
      //   init_state.rot = rotation;
      //   init_state.grav = S2(-mean_acc / mean_acc.norm() * G_m_s2);
      //   kf_state.change_x(init_state); //将初始化状态传入esekfom.hpp中的x_
      //   init_iter_num++;
      //   std::cout<<"gravity_init_success"<<std::endl;
      // }
      cov_acc *= pow(G_m_s2 / mean_acc.norm(), 2);//在上面IMU_init()基础上乘上缩放系数
      imu_need_init_ = false;

      cov_acc = cov_acc_scale;
      cov_gyr = cov_gyr_scale;
      ROS_INFO("IMU Initial Done");
      // ROS_INFO("IMU Initial Done: Gravity: %.4f %.4f %.4f %.4f; state.bias_g: %.4f %.4f %.4f; acc covarience: %.8f %.8f %.8f; gry covarience: %.8f %.8f %.8f",\
      //          imu_state.grav[0], imu_state.grav[1], imu_state.grav[2], mean_acc.norm(), cov_bias_gyr[0], cov_bias_gyr[1], cov_bias_gyr[2], cov_acc[0], cov_acc[1], cov_acc[2], cov_gyr[0], cov_gyr[1], cov_gyr[2]);
      fout_imu.open(DEBUG_FILE_DIR("imu.txt"),ios::out);
    }

    return;
  }

  UndistortPcl(meas, kf_state, *cur_pcl_un_);//点云去畸变**

  t2 = omp_get_wtime();
  t3 = omp_get_wtime();
  
  // cout<<"[ IMU Process ]: Time: "<<t3 - t1<<endl;
}
