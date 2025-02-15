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

#include "Map.h"

#include <mutex>

namespace ORB_SLAM3
{

//静态成员变量，所有对象共享这个变量
//由于是在外部定义，因此不需要再次使用static关键字
long unsigned int Map::nNextId = 0;

Map::Map()
    : mnMaxKFid(0), mnBigChangeIdx(0), mbImuInitialized(false), mnMapChange(0), mpFirstRegionKF(static_cast<KeyFrame *>(NULL)),
    mbFail(false), mIsInUse(false), mHasTumbnail(false), mbBad(false), mnMapChangeNotified(0), mbIsInertial(false), mbIMU_BA1(false), mbIMU_BA2(false)
{
    mnId = nNextId++;
    mThumbnail = static_cast<GLubyte *>(NULL);
}

/**
 * @brief mnInitKFid 初始化关键帧ID。记录创建地图时第一个关键帧的ID。
 * @brief mnMaxKFid 当前地图中最大的关键帧ID。用于跟踪地图中的关键帧ID范围。
 * @brief mnBigChangeIdx 大变化索引，用于记录地图发生重大变化的次数。
 * @brief mIsInUse 指示当前地图是否正在使用。
 * @brief mHasTumbnail 指示地图是否有缩略图。缩略图通常用于可视化和调试。
 * @brief mbBad 指示地图是否被标记为不良地图（Bad Map）。不良地图可能包含错误或不完整的数据。
 * @brief mbImuInitialized 指示地图中的IMU是否已初始化。IMU初始化对于融合视觉和惯性数据至关重要。
 * @brief mpFirstRegionKF 指向第一个区域关键帧的指针。用于区域管理和定位。
 * @brief mnMapChange 地图变化计数器，记录地图被修改的次数。
 * @brief mbFail 指示地图构建过程中是否出现失败。
 * @brief mnMapChangeNotified 记录已通知的地图变化次数。用于同步和协调。
 * @brief mbIsInertial 指示地图是否包含惯性信息。
 * @brief mbIMU_BA1 用于标记IMU数据的批处理调整状态。
 * @brief mbIMU_BA2 用于标记IMU数据的批处理调整状态。
 * @brief mnId 地图ID。每个地图实例都有一个唯一的ID，由静态变量nNextId生成。
 * @brief mThumbnail 指向地图缩略图的指针。缩略图用于可视化地图内容。
 */

Map::Map(int initKFid)
    : mnInitKFid(initKFid), mnMaxKFid(initKFid), /*mnLastLoopKFid(initKFid),*/ mnBigChangeIdx(0), mIsInUse(false),
    mHasTumbnail(false), mbBad(false), mbImuInitialized(false), mpFirstRegionKF(static_cast<KeyFrame *>(NULL)),
    mnMapChange(0), mbFail(false), mnMapChangeNotified(0), mbIsInertial(false), mbIMU_BA1(false), mbIMU_BA2(false)
{
    // 先赋值，再递增 nNextId
    mnId = nNextId++;
    // Glubyte 为无符号 字节类型 设置为NULL 表示 mThumbnail 没有被分配实际的内存
    mThumbnail = static_cast<GLubyte *>(NULL);
}

Map::~Map()
{
    // TODO: erase all points from memory
    mspMapPoints.clear();

    // TODO: erase all keyframes from memory
    mspKeyFrames.clear();

    if (mThumbnail)
        delete mThumbnail;
    mThumbnail = static_cast<GLubyte *>(NULL);

    mvpReferenceMapPoints.clear();
    mvpKeyFrameOrigins.clear();
}

// 在地图中插入关键帧,同时更新关键帧的最大id
void Map::AddKeyFrame(KeyFrame *pKF)
{
    unique_lock<mutex> lock(mMutexMap);
    if (mspKeyFrames.empty())
    {
        cout << "First KF:" << pKF->mnId << "; Map init KF:" << mnInitKFid << endl;
        mnInitKFid = pKF->mnId;
        mpKFinitial = pKF;
        mpKFlowerID = pKF;
    }
    mspKeyFrames.insert(pKF);
    if (pKF->mnId > mnMaxKFid)
    {
        mnMaxKFid = pKF->mnId;
    }
    if (pKF->mnId < mpKFlowerID->mnId)
    {
        mpKFlowerID = pKF;
    }
}

void Map::AddMapPoint(MapPoint *pMP)
{
    unique_lock<mutex> lock(mMutexMap);
    mspMapPoints.insert(pMP);
}

void Map::SetImuInitialized()
{
    unique_lock<mutex> lock(mMutexMap);
    mbImuInitialized = true;
}

bool Map::isImuInitialized()
{
    unique_lock<mutex> lock(mMutexMap);
    return mbImuInitialized;
}

void Map::EraseMapPoint(MapPoint *pMP)
{
    unique_lock<mutex> lock(mMutexMap);
    mspMapPoints.erase(pMP);

    // 下面是作者加入的注释. 实际上只是从std::set中删除了地图点的指针, 原先地图点
    // 占用的内存区域并没有得到释放
    // TODO: This only erase the pointer.
    // Delete the MapPoint
}

void Map::EraseKeyFrame(KeyFrame *pKF)
{
    unique_lock<mutex> lock(mMutexMap);
    mspKeyFrames.erase(pKF);
    if (mspKeyFrames.size() > 0)
    {
        if (pKF->mnId == mpKFlowerID->mnId)
        {
            vector<KeyFrame *> vpKFs = vector<KeyFrame *>(mspKeyFrames.begin(), mspKeyFrames.end());
            sort(vpKFs.begin(), vpKFs.end(), KeyFrame::lId);
            mpKFlowerID = vpKFs[0];
        }
    }
    else
    {
        mpKFlowerID = 0;
    }

    // TODO: This only erase the pointer.
    // Delete the MapPoint
}

/*
 * @brief 设置参考MapPoints，将用于DrawMapPoints函数画图
 * 设置参考地图点用于绘图显示局部地图点（红色）
 * @param vpMPs Local MapPoints
 */
void Map::SetReferenceMapPoints(const vector<MapPoint *> &vpMPs)
{
    unique_lock<mutex> lock(mMutexMap);
    mvpReferenceMapPoints = vpMPs;
}

void Map::InformNewBigChange()
{
    unique_lock<mutex> lock(mMutexMap);
    mnBigChangeIdx++;
}

int Map::GetLastBigChangeIdx()
{
    unique_lock<mutex> lock(mMutexMap);
    return mnBigChangeIdx;
}

// 获取地图中的所有关键帧
vector<KeyFrame *> Map::GetAllKeyFrames()
{
    unique_lock<mutex> lock(mMutexMap);
    return vector<KeyFrame *>(mspKeyFrames.begin(), mspKeyFrames.end());
}

// 获取地图中的所有地图点
vector<MapPoint *> Map::GetAllMapPoints()
{
    unique_lock<mutex> lock(mMutexMap);
    return vector<MapPoint *>(mspMapPoints.begin(), mspMapPoints.end());
}

// 获取地图点数目
long unsigned int Map::MapPointsInMap()
{
    unique_lock<mutex> lock(mMutexMap);
    return mspMapPoints.size();
}

// 获取地图中的关键帧数目
long unsigned int Map::KeyFramesInMap()
{
    unique_lock<mutex> lock(mMutexMap);
    return mspKeyFrames.size();
}

// 获取参考地图点
vector<MapPoint *> Map::GetReferenceMapPoints()
{
    unique_lock<mutex> lock(mMutexMap);
    return mvpReferenceMapPoints;
}

long unsigned int Map::GetId()
{
    return mnId;
}
long unsigned int Map::GetInitKFid()
{
    unique_lock<mutex> lock(mMutexMap);
    return mnInitKFid;
}

void Map::SetInitKFid(long unsigned int initKFif)
{
    unique_lock<mutex> lock(mMutexMap);
    mnInitKFid = initKFif;
}

long unsigned int Map::GetMaxKFid()
{
    unique_lock<mutex> lock(mMutexMap);
    return mnMaxKFid;
}

KeyFrame *Map::GetOriginKF()
{
    return mpKFinitial;
}

void Map::SetCurrentMap()
{
    mIsInUse = true;
}

void Map::SetStoredMap()
{
    mIsInUse = false;
}

void Map::clear()
{
    //    for(set<MapPoint*>::iterator sit=mspMapPoints.begin(), send=mspMapPoints.end(); sit!=send; sit++)
    //        delete *sit;

    for (set<KeyFrame *>::iterator sit = mspKeyFrames.begin(), send = mspKeyFrames.end(); sit != send; sit++)
    {
        KeyFrame *pKF = *sit;
        pKF->UpdateMap(static_cast<Map *>(NULL));
        //        delete *sit;
    }

    mspMapPoints.clear();
    mspKeyFrames.clear();
    mnMaxKFid = mnInitKFid;
    mbImuInitialized = false;
    mvpReferenceMapPoints.clear();
    mvpKeyFrameOrigins.clear();
    mbIMU_BA1 = false;
    mbIMU_BA2 = false;
}

bool Map::IsInUse()
{
    return mIsInUse;
}

void Map::SetBad()
{
    mbBad = true;
}

bool Map::IsBad()
{
    return mbBad;
}

// 恢复尺度及重力方向
/** imu在localmapping中初始化，LocalMapping::InitializeIMU中使用，误差包含三个残差与两个偏置
 * 地图融合时也会使用
 * @param R 初始化时为Rgw
 * @param s 尺度
 * @param bScaledVel 将尺度更新到速度
 * @param t 默认cv::Mat::zeros(cv::Size(1,3),CV_32F)
 */
void Map::ApplyScaledRotation(const Sophus::SE3f &T, const float s, const bool bScaledVel)
{
    unique_lock<mutex> lock(mMutexMap);

    // Body position (IMU) of first keyframe is fixed to (0,0,0)
    Sophus::SE3f Tyw = T;
    Eigen::Matrix3f Ryw = Tyw.rotationMatrix();
    Eigen::Vector3f tyw = Tyw.translation();

    for (set<KeyFrame *>::iterator sit = mspKeyFrames.begin(); sit != mspKeyFrames.end(); sit++)
    {
        // 更新关键帧位姿
        /**
         * | Rw2w1  tw2w1 |   *   | Rw1c  s*tw1c  |     =    |  Rw2c     s*Rw2w1*tw1c + tw2w1  |
         * |   0      1   |       |  0       1    |          |   0                1            |
         * 这么做比正常乘在旋转上少了个s，后面不需要这个s了，因为所有mp在下面已经全部转到了w2坐标系下，不存在尺度变化了
         * 
         * | s*Rw2w1  tw2w1 |   *   | Rw1c    tw1c  |     =    |  s*Rw2c     s*Rw2w1*tw1c + tw2w1  |
         * |   0        1   |       |  0       1    |          |     0                1            |
         */
        KeyFrame *pKF = *sit;
        Sophus::SE3f Twc = pKF->GetPoseInverse();
        Twc.translation() *= s;

        // |  Ryc     s*Ryw*twc + tyw  |
        // |   0           1           |
        Sophus::SE3f Tyc = Tyw * Twc;
        Sophus::SE3f Tcy = Tyc.inverse();
        pKF->SetPose(Tcy);
        // 更新关键帧速度
        Eigen::Vector3f Vw = pKF->GetVelocity();
        if (!bScaledVel)
            pKF->SetVelocity(Ryw * Vw);
        else
            pKF->SetVelocity(Ryw * Vw * s);
    }
    for (set<MapPoint *>::iterator sit = mspMapPoints.begin(); sit != mspMapPoints.end(); sit++)
    {
        // 更新每一个mp在世界坐标系下的坐标
        MapPoint *pMP = *sit;
        if (!pMP || pMP->isBad())
            continue;
        pMP->SetWorldPos(s * Ryw * pMP->GetWorldPos() + tyw);
        pMP->UpdateNormalAndDepth();
    }
    mnMapChange++;
}

void Map::SetInertialSensor()
{
    unique_lock<mutex> lock(mMutexMap);
    mbIsInertial = true;
}

bool Map::IsInertial()
{
    unique_lock<mutex> lock(mMutexMap);
    // 将mbIsInertial设置为true,将其设置为imu属性,以后的跟踪和预积分将和这个标志有关
    return mbIsInertial;
}

void Map::SetIniertialBA1()
{
    unique_lock<mutex> lock(mMutexMap);
    mbIMU_BA1 = true;
}

void Map::SetIniertialBA2()
{
    unique_lock<mutex> lock(mMutexMap);
    mbIMU_BA2 = true;
}

bool Map::GetIniertialBA1()
{
    unique_lock<mutex> lock(mMutexMap);
    return mbIMU_BA1;
}

bool Map::GetIniertialBA2()
{
    unique_lock<mutex> lock(mMutexMap);
    return mbIMU_BA2;
}

void Map::ChangeId(long unsigned int nId)
{
    mnId = nId;
}

unsigned int Map::GetLowerKFID()
{
    unique_lock<mutex> lock(mMutexMap);
    if (mpKFlowerID)
    {
        return mpKFlowerID->mnId;
    }
    return 0;
}

int Map::GetMapChangeIndex()
{
    unique_lock<mutex> lock(mMutexMap);
    return mnMapChange;
}

void Map::IncreaseChangeIndex()
{
    unique_lock<mutex> lock(mMutexMap);
    mnMapChange++;
}

int Map::GetLastMapChange()
{
    unique_lock<mutex> lock(mMutexMap);
    return mnMapChangeNotified;
}

void Map::SetLastMapChange(int currentChangeId)
{
    unique_lock<mutex> lock(mMutexMap);
    mnMapChangeNotified = currentChangeId;
}

/** 预保存，也就是把想保存的信息保存到备份的变量中
 * @param spCams 相机
 */
void Map::PreSave(std::set<GeometricCamera *> &spCams)
{
    int nMPWithoutObs = 0;  // 统计用
    // 1. 剔除一下无效观测
    for (MapPoint *pMPi : mspMapPoints)
    {
        if (!pMPi || pMPi->isBad())
            continue;

        if (pMPi->GetObservations().size() == 0)
        {
            nMPWithoutObs++;
        }
        map<KeyFrame *, std::tuple<int, int>> mpObs = pMPi->GetObservations();
        for (map<KeyFrame *, std::tuple<int, int>>::iterator it = mpObs.begin(), end = mpObs.end(); it != end; ++it)
        {
            if (!it->first || it->first->GetMap() != this || it->first->isBad())
            {
                pMPi->EraseObservation(it->first, false);
            }
        }
    }

    // Saves the id of KF origins
    // 2. 保存最开始的帧的id，貌似一个map的mvpKeyFrameOrigins里面只有一个，可以验证一下
    mvBackupKeyFrameOriginsId.clear();
    mvBackupKeyFrameOriginsId.reserve(mvpKeyFrameOrigins.size());
    for (int i = 0, numEl = mvpKeyFrameOrigins.size(); i < numEl; ++i)
    {
        mvBackupKeyFrameOriginsId.push_back(mvpKeyFrameOrigins[i]->mnId);
    }

    // Backup of MapPoints
    // 3. 保存一下对应的mp
    mvpBackupMapPoints.clear();
    for (MapPoint *pMPi : mspMapPoints)
    {
        if (!pMPi || pMPi->isBad())
            continue;

        mvpBackupMapPoints.push_back(pMPi);
        pMPi->PreSave(mspKeyFrames, mspMapPoints);
    }

    // Backup of KeyFrames
    // 4. 保存一下对应的KF
    mvpBackupKeyFrames.clear();
    for (KeyFrame *pKFi : mspKeyFrames)
    {
        if (!pKFi || pKFi->isBad())
            continue;

        mvpBackupKeyFrames.push_back(pKFi);
        pKFi->PreSave(mspKeyFrames, mspMapPoints, spCams);
    }

    // 保存一些id
    mnBackupKFinitialID = -1;
    if (mpKFinitial)
    {
        mnBackupKFinitialID = mpKFinitial->mnId;
    }

    mnBackupKFlowerID = -1;
    if (mpKFlowerID)
    {
        mnBackupKFlowerID = mpKFlowerID->mnId;
    }
}

/** 后加载，也就是把备份的变量恢复到正常变量中
 * @param spCams 相机
 */
void Map::PostLoad(KeyFrameDatabase *pKFDB, ORBVocabulary *pORBVoc /*, map<long unsigned int, KeyFrame*>& mpKeyFrameId*/, map<unsigned int, GeometricCamera *> &mpCams)
{
    std::copy(mvpBackupMapPoints.begin(), mvpBackupMapPoints.end(), std::inserter(mspMapPoints, mspMapPoints.begin()));
    std::copy(mvpBackupKeyFrames.begin(), mvpBackupKeyFrames.end(), std::inserter(mspKeyFrames, mspKeyFrames.begin()));

    // 1. 恢复map中的mp，注意此时mp中只恢复了保存的量
    map<long unsigned int, MapPoint *> mpMapPointId;
    for (MapPoint *pMPi : mspMapPoints)
    {
        if (!pMPi || pMPi->isBad())
            continue;

        pMPi->UpdateMap(this);
        mpMapPointId[pMPi->mnId] = pMPi;
    }

    // 2. 恢复map中的kf，注意此时kf中只恢复了保存的量
    map<long unsigned int, KeyFrame *> mpKeyFrameId;
    for (KeyFrame *pKFi : mspKeyFrames)
    {
        if (!pKFi || pKFi->isBad())
            continue;

        pKFi->UpdateMap(this);
        pKFi->SetORBVocabulary(pORBVoc);
        pKFi->SetKeyFrameDatabase(pKFDB);
        mpKeyFrameId[pKFi->mnId] = pKFi;
    }

    // References reconstruction between different instances
    // 3. 使用mp中的备份变量恢复正常变量
    for (MapPoint *pMPi : mspMapPoints)
    {
        if (!pMPi || pMPi->isBad())
            continue;

        pMPi->PostLoad(mpKeyFrameId, mpMapPointId);
    }

    // 4. 使用kf中的备份变量恢复正常变量
    for (KeyFrame *pKFi : mspKeyFrames)
    {
        if (!pKFi || pKFi->isBad())
            continue;

        pKFi->PostLoad(mpKeyFrameId, mpMapPointId, mpCams);
        pKFDB->add(pKFi);
    }

    // 5. 恢复ID
    if (mnBackupKFinitialID != -1)
    {
        mpKFinitial = mpKeyFrameId[mnBackupKFinitialID];
    }

    if (mnBackupKFlowerID != -1)
    {
        mpKFlowerID = mpKeyFrameId[mnBackupKFlowerID];
    }

    mvpKeyFrameOrigins.clear();
    mvpKeyFrameOrigins.reserve(mvBackupKeyFrameOriginsId.size());
    for (int i = 0; i < mvBackupKeyFrameOriginsId.size(); ++i)
    {
        mvpKeyFrameOrigins.push_back(mpKeyFrameId[mvBackupKeyFrameOriginsId[i]]);
    }

    mvpBackupMapPoints.clear();
}

} // namespace ORB_SLAM3
