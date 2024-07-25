/**
* This file is part of ORB-SLAM3
*
* Copyright (C) 2017-2021 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
* Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
*
* ORB-SLAM3 is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
* License as published by the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
* the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with ORB-SLAM3.
* If not, see <http://www.gnu.org/licenses/>.
*/


#ifndef SYSTEM_H
#define SYSTEM_H


#include <unistd.h>
#include<stdio.h>
#include<stdlib.h>
#include<string>
#include<thread>
#include<opencv2/core/core.hpp>

#include "Tracking.h"
#include "FrameDrawer.h"
#include "MapDrawer.h"
#include "Atlas.h"
#include "LocalMapping.h"
#include "LoopClosing.h"
#include "KeyFrameDatabase.h"
#include "ORBVocabulary.h"
#include "Viewer.h"
#include "ImuTypes.h"
#include "Settings.h"


namespace ORB_SLAM3
{

class Verbose
{
public:
    enum eLevel
    {
        VERBOSITY_QUIET=0,
        VERBOSITY_NORMAL=1,
        VERBOSITY_VERBOSE=2,
        VERBOSITY_VERY_VERBOSE=3,
        VERBOSITY_DEBUG=4
    };

    static eLevel th;

public:
    static void PrintMess(std::string str, eLevel lev)
    {
        if(lev <= th)
            cout << str << endl;
    }

    static void SetTh(eLevel _th)
    {
        th = _th;
    }
};

class Viewer;
class FrameDrawer;
class MapDrawer;
class Atlas;
class Tracking;
class LocalMapping;
class LoopClosing;
class Settings;

class System
{
public:
    // Input sensor 
    // 枚举数据类型
    enum eSensor{
        MONOCULAR=0,
        STEREO=1,
        RGBD=2,
        IMU_MONOCULAR=3,
        IMU_STEREO=4,
        IMU_RGBD=5,
    };

    // File type
    enum FileType{
        TEXT_FILE=0,
        BINARY_FILE=1,
    };

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    // 初始化 SLAM 系统。它启动本地建图、环路闭合和查看器线程。
    // 参数类型:
    //      ORB词汇表路径
    //      配置文件路径
    //      是否启用可视化
    //      指定从哪一帧开始初始化 默认为0
    //      数据集序列路径，如果提供，将从指定路径在家数据集序列，供系统处理
    System(const string &strVocFile, const string &strSettingsFile, const eSensor sensor, const bool bUseViewer = true, const int initFr = 0, const string &strSequence = std::string());

    // Proccess the given stereo frame. Images must be synchronized and rectified.
    // Input images: RGB (CV_8UC3) or grayscale (CV_8U). RGB is converted to grayscale.
    // Returns the camera pose (empty if tracking fails)
    // 处理给定的立体图像帧；图像必须 时间上同步 完成矫正 (左右相机进行几何变换，使得像素对齐，为了简化立体匹配，使得对应点只在水平方向上位移)
    // 输入图像为 RGB 或者 灰度图像；RGB会被转换成灰度图像
    //          CV_8UC3 三个通道，每个通道8位无符号整数代表 
    //          CV_8U   单通道，8位无符号整数代表
    // 返回 相机位姿，如果跟踪失败，则返回空
    Sophus::SE3f TrackStereo(const cv::Mat &imLeft, const cv::Mat &imRight, const double &timestamp, const vector<IMU::Point>& vImuMeas = vector<IMU::Point>(), string filename="");

    // Process the given rgbd frame. Depthmap must be registered to the RGB frame.
    // Input image: RGB (CV_8UC3) or grayscale (CV_8U). RGB is converted to grayscale.
    // Input depthmap: Float (CV_32F).
    // Returns the camera pose (empty if tracking fails).
    // 处理给定的 RGB-D 图像帧。深度图必须与 RGB 图像对齐
    // 输入图像：RGB（CV_8UC3）或灰度（CV_8U）。RGB 图像将转换为灰度图像
    // 输入深度图：浮点型（CV_32F）
    //          CV_32F 单通道，每个通道32位无符号整数代表
    // 返回相机位姿（如果跟踪失败，则返回空）
    Sophus::SE3f TrackRGBD(const cv::Mat &im, const cv::Mat &depthmap, const double &timestamp, const vector<IMU::Point>& vImuMeas = vector<IMU::Point>(), string filename="");

    // Proccess the given monocular frame and optionally imu data
    // Input images: RGB (CV_8UC3) or grayscale (CV_8U). RGB is converted to grayscale.
    // Returns the camera pose (empty if tracking fails).
    // 处理给定的单目帧和可选的 imu 数据
    // 输入图像：RGB（CV_8UC3）或灰度（CV_8U）。RGB 转换为灰度。
    // 返回相机姿势（如果跟踪失败则为空）。
    Sophus::SE3f TrackMonocular(const cv::Mat &im, const double &timestamp, const vector<IMU::Point>& vImuMeas = vector<IMU::Point>(), string filename="");


    // This stops local mapping thread (map building) and performs only camera tracking.
    // 停止局部建图线程，仅执行相机追踪
    void ActivateLocalizationMode();
    // This resumes local mapping thread and performs SLAM again.
    // 恢复局部建图线程，不光进行相机跟踪，还会同时进行地图构建和优化工作
    void DeactivateLocalizationMode();

    // Returns true if there have been a big map change (loop closure, global BA)
    // since last call to this function
    // 自上次调用这个函数以来，地图是否发生了重大变化，例如回环检测，整体BA
    // 返回 真 / 假
    bool MapChanged();

    // Reset the system (clear Atlas or the active map)
    // Atlas 维护了全局地图，存储了所有的关键帧和地图点 Rest会清空Atlas的地图，恢复到初始状态，这样系统可以重新开始运行
    void Reset();
    // 仅仅有活跃地图被重置
    void ResetActiveMap();

    // All threads will be requested to finish.
    // It waits until all threads have finished.
    // This function must be called before saving the trajectory.
    // 关闭所有的线程，必须在保存轨迹之前调用这个函数
    void Shutdown();
    bool isShutDown();

    // Save camera trajectory in the TUM RGB-D dataset format.
    // Only for stereo and RGB-D. This method does not work for monocular.
    // Call first Shutdown()
    // See format details at: http://vision.in.tum.de/data/datasets/rgbd-dataset
    // 以 TUM RGB-D 数据集格式保存相机轨迹。
    // 仅适用于立体相机和 RGB-D 相机。此方法不适用于单目相机。
    // 请先调用 Shutdown()
    // 详细格式请参见：http://vision.in.tum.de/data/datasets/rgbd-dataset
    void SaveTrajectoryTUM(const string &filename);

    // Save keyframe poses in the TUM RGB-D dataset format.
    // This method works for all sensor input.
    // Call first Shutdown()
    // See format details at: http://vision.in.tum.de/data/datasets/rgbd-dataset
    // 以 TUM RGB-D 数据集格式保存关键帧姿态。
    // 此方法适用于所有传感器输入。
    // 请先调用 Shutdown()
    // 详细格式请参见：http://vision.in.tum.de/data/datasets/rgbd-dataset
    void SaveKeyFrameTrajectoryTUM(const string &filename);

    // EuRoC 数据集
    void SaveTrajectoryEuRoC(const string &filename);
    void SaveKeyFrameTrajectoryEuRoC(const string &filename);
    void SaveTrajectoryEuRoC(const string &filename, Map* pMap);
    void SaveKeyFrameTrajectoryEuRoC(const string &filename, Map* pMap);

    // Save data used for initialization debug
    // 保存调试数据，传入参数为初始化索引，用于标识保存的调试数据
    // 在系统进行调试和初始化时，保存相关数据以便后续分析和调试
    void SaveDebugData(const int &iniIdx);

    // Save camera trajectory in the KITTI dataset format.
    // Only for stereo and RGB-D. This method does not work for monocular.
    // Call first Shutdown()
    // See format details at: http://www.cvlibs.net/datasets/kitti/eval_odometry.php
    void SaveTrajectoryKITTI(const string &filename);

    // TODO: Save/Load functions
    // SaveMap(const string &filename);
    // LoadMap(const string &filename);

    // Information from most recent processed frame
    // You can call this right after TrackMonocular (or stereo or RGBD)
    // 获取最近处理帧的跟踪状态，可以在调用Track...函数后立即调用这个函数
    // 返回为一个整数，表示跟踪状态
    int GetTrackingState();

    // 获取当前跟踪到的地图点
    // 返回值为一个地图点的向量
    std::vector<MapPoint*> GetTrackedMapPoints();

    // 获取当前跟踪到的 没有匹配的关键点
    // 返回 没有匹配的关键点
    std::vector<cv::KeyPoint> GetTrackedKeyPointsUn();

    // For debugging
    // 获取IMU初始化后到当前的时间
    double GetTimeFromIMUInit();

    // 检查是否跟丢
    bool isLost();

    // 检查系统是否完成所有的操作
    bool isFinished();

    // 切换一个数据集进行处理
    void ChangeDataset();

    // 获取当前图像缩放比例
    // 返回图像缩放比例
    float GetImageScale();

#ifdef REGISTER_TIMES
    void InsertRectTime(double& time);
    void InsertResizeTime(double& time);
    void InsertTrackTime(double& time);
#endif

private:
    // 存储Atlas地图
    void SaveAtlas(int type);
    // 加载Atlas地图
    bool LoadAtlas(int type);

    // 计算CheckSum，可以用来检测文件是否发生了变化或损坏
    string CalculateCheckSum(string filename, int type);

    // Input sensor
    // 枚举数据类型，定义一组命名的常量，定义了不同类型的传感器
    eSensor mSensor;

    // ORB vocabulary used for place recognition and feature matching.
    // 指针类型 
    // 用来位置识别 和 特征匹配
    ORBVocabulary* mpVocabulary;

    // KeyFrame database for place recognition (relocalization and loop detection).
    // 指针类型
    // 用来 位置识别 (重定位 回环检测)
    KeyFrameDatabase* mpKeyFrameDatabase;

    // Map structure that stores the pointers to all KeyFrames and MapPoints.
    //Map* mpMap;

    // 指针
    // 跟全局地图有关
    Atlas* mpAtlas;

    // Tracker. It receives a frame and computes the associated camera pose.
    // It also decides when to insert a new keyframe, create some new MapPoints and
    // performs relocalization if tracking fails.
    // 追踪指针
    // 跟踪器：接受帧 并 计算相关的 位姿
    // 决定 什么时候插入新的 关键帧、创建一些新的地图点
    // 并且跟踪失败的时候重定位
    Tracking* mpTracker;

    // Local Mapper. It manages the local map and performs local bundle adjustment.
    // 局部建图指针
    // 局部建图起，管理局部地图 执行 局部BA
    LocalMapping* mpLocalMapper;

    // Loop Closer. It searches loops with every new keyframe. If there is a loop it performs
    // a pose graph optimization and full bundle adjustment (in a new thread) afterwards.
    // 闭环指针
    // 回环检测器，在每个新的关键帧中搜索回环
    // 如果存在回环，它将在另一个线程中执行 位姿图优化 和 全局BA
    LoopClosing* mpLoopCloser;

    // The viewer draws the map and the current camera pose. It uses Pangolin.
    // 利用Pangolin 绘制当前地图和当前相机位姿
    Viewer* mpViewer;
    FrameDrawer* mpFrameDrawer;
    MapDrawer* mpMapDrawer;

    // System threads: Local Mapping, Loop Closing, Viewer.
    // The Tracking thread "lives" in the main execution thread that creates the System object.
    // 线程指针
    // 创建三个线程，分别是局部建图，回环检测 和 视图
    std::thread* mptLocalMapping;
    std::thread* mptLoopClosing;
    std::thread* mptViewer;

    // Reset flag
    // 互斥锁，多线程中防止同时修改数据，从而避免数据竞争
    // 看名字，可以发现主要是是对下面两个变量进行锁定，防止多线程访问
    // 两个变量是 重置 相关 都是bool类型
    std::mutex mMutexReset;
    bool mbReset;
    bool mbResetActiveMap;

    // Change mode flags
    // 锁定是否要激活 定位模式 都是bool类型
    std::mutex mMutexMode;
    bool mbActivateLocalizationMode;
    bool mbDeactivateLocalizationMode;

    // Shutdown flag
    bool mbShutDown;

    // Tracking state
    int mTrackingState;

    // 追踪到的地图点
    std::vector<MapPoint*> mTrackedMapPoints;
    
    // 没有追踪到的地图点
    std::vector<cv::KeyPoint> mTrackedKeyPointsUn;

    // 状态互斥锁
    std::mutex mMutexState;

    //存储 从文件中 加载 地图数据(Atlas) 的文件路径
    string mStrLoadAtlasFromFile;
    
    //存储 将 地图数据 保存到文件 的文件路径
    string mStrSaveAtlasToFile;

    // 加载ORB词汇表的文件路径
    string mStrVocabularyFilePath;

    // 系统配置指针
    Settings* settings_;
};

}// namespace ORB_SLAM

#endif // SYSTEM_H
