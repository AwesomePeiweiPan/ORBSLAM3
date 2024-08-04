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


#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>
#include <ctime>
#include <sstream>

#include<opencv2/core/core.hpp>

#include<System.h>
#include "ImuTypes.h"

#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>

using namespace std;

//加载图像
void LoadImages(const string &strImagePath, const string &strPathTimes,
                vector<string> &vstrImages, vector<double> &vTimeStamps);

//加载IMU
void LoadIMU(const string &strImuPath, vector<double> &vTimeStamps, vector<cv::Point3f> &vAcc, vector<cv::Point3f> &vGyro);

//全局变量，记录总的跟踪时间
double ttrack_tot = 0;

int main(int argc, char *argv[])
{

    //输入参数解释
    //  1.程序名
    //  2.ORB词汇文件路径：100多兆
    //  3.EuRoC.yaml文件路径，里面设置了相机参数，IMU参数，基本参数等
    //  4.图像序列路径             
    //  5.图像序列对应的时间戳(4和5代表一个序列，可以有多个序列)        
    //  6.输出保存文件地址(可选)
    if(argc < 5)             
    {  
        cerr << endl << "Usage: ./mono_inertial_euroc path_to_vocabulary path_to_settings path_to_sequence_folder_1 path_to_times_file_1 (path_to_image_folder_2 path_to_times_file_2 ... path_to_image_folder_N path_to_times_file_N) " << endl;
        return 1;
    }

    //根据输入 给出输入序列的个数
    const int num_seq = (argc-3)/2;
    cout << "num_seq = " << num_seq << endl;
    //如果有输出保存文件的地址，则输出这个文件名字
    bool bFileName= (((argc-3) % 2) == 1);
    string file_name;
    if (bFileName)
    {
        file_name = string(argv[argc-1]);
        cout << "file name: " << file_name << endl;
    }

    // Load all sequences:
    int seq;
    vector< vector<string> > vstrImageFilenames;    //图像文件名
    vector< vector<double> > vTimestampsCam;        //图像时间戳
    vector< vector<cv::Point3f> > vAcc, vGyro;      //加速度计，陀螺仪
    vector< vector<double> > vTimestampsImu;        //IMU时间戳
    vector<int> nImages;                            //图像序列
    vector<int> nImu;                               //IMU序列
    vector<int> first_imu(num_seq,0);                //记录第一帧图像时间戳 最接近的 imu时间戳索引
                                                    //如果 num_seq = 3, first_imu=[0,0,0];

    //将二维的数组 外部向量大小调整为1 内部向量仍然为空大小为0
    //例子: vstrImageFilenames.size()=1; vstrImageFilenames[0].size()=0;
    vstrImageFilenames.resize(num_seq);    
    vTimestampsCam.resize(num_seq);
    vAcc.resize(num_seq);
    vGyro.resize(num_seq);
    vTimestampsImu.resize(num_seq);
    nImages.resize(num_seq);
    nImu.resize(num_seq);

    int tot_images = 0;
    //这里就一个图像序列
    for (seq = 0; seq<num_seq; seq++)
    {
        cout << "Loading images for sequence " << seq << "...";

        //赋值图像序列地址
        string pathSeq(argv[(2*seq) + 3]);
        //赋值图像时间戳序列地址
        string pathTimeStamps(argv[(2*seq) + 4]);

        //更精确的地址
        string pathCam0 = pathSeq + "/mav0/cam0/data";
        string pathImu = pathSeq + "/mav0/imu0/data.csv";

        //将每张图像的地址，每张图像的时间戳，全部都存入 vstrImageFilenames[0], vTimestampsCam[0] 中
        LoadImages(pathCam0, pathTimeStamps, vstrImageFilenames[seq], vTimestampsCam[seq]);
        cout << "LOADED!" << endl;

        //将IMU数据 的 时间，加速度计数据，陀螺仪数据 存入 vTimestampsImu[0], vAcc[0], vGyro[0])
        cout << "Loading IMU for sequence " << seq << "...";
        LoadIMU(pathImu, vTimestampsImu[seq], vAcc[seq], vGyro[seq]);
        cout << "LOADED!" << endl;

        //记录当前序列的图像数据量和IMU数据量；比如第0个序列100张图，nImages[0]=100
        nImages[seq] = vstrImageFilenames[seq].size();
        //累积到当前序列的图像总数量
        tot_images += nImages[seq];
        //比如第0个序列100个IMU序列，nImup[0]=100
        nImu[seq] = vTimestampsImu[seq].size();

        //防止没加载上
        if((nImages[seq]<=0)||(nImu[seq]<=0))
        {
            cerr << "ERROR: Failed to load images or IMU for sequence" << seq << endl;
            return 1;
        }

        //找到 第一张 image 的 时间上 最邻近的 前一个 imu数据: 比如 first_imu[seq=0]=64
        while(vTimestampsImu[seq][first_imu[seq]]<=vTimestampsCam[seq][0])
            first_imu[seq]++;
        first_imu[seq]--; // first imu measurement to be considered

    }

    // 每张图像的处理时间
    vector<float> vTimesTrack;
    vTimesTrack.resize(tot_images);

    cout.precision(17); //输出流确保 17位 有效数字，符合 double类型 精度
    
    // !!!!!!!!!!!!!!!!!!!SLAM 初始化!!!!!!!!!!!!!!!!!!!
    //SLAM系统创建 指定: 词汇表路径 + EuRoC.yaml文件路径 + 单目模式 + 可视化
    ORB_SLAM3::System SLAM(argv[1],argv[2],ORB_SLAM3::System::IMU_MONOCULAR, false);
    //获取图像缩放比例
    float imageScale = SLAM.GetImageScale();

    double t_resize = 0.f;
    double t_track = 0.f;

    //初始化图像计数器
    int proccIm=0;
    for (seq = 0; seq<num_seq; seq++)
    {                                                                                       

        // ! Main loop 主循环
        cv::Mat im;  //存储图像数据                                         
        vector<ORB_SLAM3::IMU::Point> vImuMeas;  //IMU数据
        proccIm = 0;
    //遍历 当前序列中 所有图像
        for(int ni=0; ni<nImages[seq]; ni++, proccIm++)
        {
            // 读取文件，IMREAD_UNCHANGED表示 读取图像的时候不改变原始的数据类型
            im = cv::imread(vstrImageFilenames[seq][ni],cv::IMREAD_UNCHANGED); //CV_LOAD_IMAGE_UNCHANGED);

            //获取图像的时间戳
            double tframe = vTimestampsCam[seq][ni];

            //查看图像是否加载成功
            if(im.empty())
            {
                cerr << endl << "Failed to load image at: "
                     <<  vstrImageFilenames[seq][ni] << endl;
                return 1;
            }

            //查看图像是否需要缩放
            if(imageScale != 1.f)
            {
                // 定义起始时间点
                #ifdef REGISTER_TIMES           //如果定义了这个宏
                    #ifdef COMPILEDWITHC11      //如果使用了C++11的标准
                        std::chrono::steady_clock::time_point t_Start_Resize = std::chrono::steady_clock::now();
                    #else
                        std::chrono::monotonic_clock::time_point t_Start_Resize = std::chrono::monotonic_clock::now();
                    #endif
                #endif
                int width = im.cols * imageScale;               //计算缩放后的图像宽度
                int height = im.rows * imageScale;              //计算缩放后的图像高度
                cv::resize(im, im, cv::Size(width, height));    //缩放图像到指定大小

                // 定义当前时间点，即图像缩放结束的时间点
                #ifdef REGISTER_TIMES
                    #ifdef COMPILEDWITHC11
                        std::chrono::steady_clock::time_point t_End_Resize = std::chrono::steady_clock::now();
                    #else
                        std::chrono::monotonic_clock::time_point t_End_Resize = std::chrono::monotonic_clock::now();
                    #endif
                    t_resize = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(t_End_Resize - t_Start_Resize).count();
                    SLAM.InsertResizeTime(t_resize);
                #endif
            }

            // 清空前一帧的IMU测量数据
            vImuMeas.clear();

            //向vImuMeans中添加 时间戳小于 相机时间戳的 IMU数据集
            //说白了，向 vImuMeans 中添加的是 这帧之前第一个 到 上一帧后面第一个 的所有IMU数据
            if(ni>0)
            {
                // cout << "t_cam " << tframe << endl;

                while(vTimestampsImu[seq][first_imu[seq]]<=vTimestampsCam[seq][ni])
                {
                    vImuMeas.push_back(ORB_SLAM3::IMU::Point(vAcc[seq][first_imu[seq]].x,vAcc[seq][first_imu[seq]].y,vAcc[seq][first_imu[seq]].z,
                                                             vGyro[seq][first_imu[seq]].x,vGyro[seq][first_imu[seq]].y,vGyro[seq][first_imu[seq]].z,
                                                             vTimestampsImu[seq][first_imu[seq]]));
                    first_imu[seq]++;
                }
            }


            // 定义追踪这一帧开始的时间点
            std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
            // Pass the image to the SLAM system
            // cout << "tframe = " << tframe << endl;


            // !!!!!!!!!!!跟踪线程作为主线程！!!!!!!!!!!!!!!!
            SLAM.TrackMonocular(im,tframe,vImuMeas); // TODO change to monocular_inertial
            // 定义结束追踪这一帧的时间点
            std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
   


            // 计算t1和t2之间的时间差
            #ifdef REGISTER_TIMES
                t_track = t_resize + std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(t2 - t1).count();
                SLAM.InsertTrackTime(t_track);
            #endif
            // 计算t1和t2之间的时间差
            double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();
            // 记录总时间
            ttrack_tot += ttrack;
            // std::cout << "ttrack: " << ttrack << std::endl;

            // 记录追踪帧时间
            vTimesTrack[ni]=ttrack;

            // 等待加载下一帧
            double T=0;
            if(ni<nImages[seq]-1)                       //当前帧 不是 序列中 最后一帧
                T = vTimestampsCam[seq][ni+1]-tframe;   //T为下一帧与当前帧的时间戳差值
            else if(ni>0)                               //当前真 是 序列中的最后一帧 且不是 第一帧
                T = tframe-vTimestampsCam[seq][ni-1];   //T为当前帧与前一帧的时间戳的差值

            if(ttrack<T)                                //如果当前帧处理时间小与 到下一帧的 两帧本来的时间
                usleep((T-ttrack)*1e6); // 1e6          //休息一会，等待下一帧
        }

        //如果不是最后一个数据集，则切数据集
        if(seq < num_seq - 1)
        {
            cout << "Changing the dataset" << endl;

            SLAM.ChangeDataset();
        }
    }

    // 停止所有的线程
    SLAM.Shutdown();

    // 保存轨迹
    if (bFileName)
    {
        const string kf_file =  "kf_" + string(argv[argc-1]) + ".txt";
        const string f_file =  "f_" + string(argv[argc-1]) + ".txt";
        SLAM.SaveTrajectoryEuRoC(f_file);
        SLAM.SaveKeyFrameTrajectoryEuRoC(kf_file);
    }
    else
    {
        SLAM.SaveTrajectoryEuRoC("CameraTrajectory.txt");
        SLAM.SaveKeyFrameTrajectoryEuRoC("KeyFrameTrajectory.txt");
    }

    return 0;
}


/**
 * @brief 加载图像以及时间戳
 * 
 * @param[in] strImagePath  已有的图像路径
 * @param[in] strPathTimes  已有的图像时间戳路径
 * @param[in] vstrImages    待输入的图像帧
 * @param[in] vTimeStamps   待输入的时间戳
 */
void LoadImages(const string &strImagePath, const string &strPathTimes,
                vector<string> &vstrImages, vector<double> &vTimeStamps)
{   
    //定义输入流 ifstream 对象,是 istream 类的派生类，用于文件的输入操作，可以从文件中读区各种类型的数据
    ifstream fTimes;
    //ifstream 的构造函数和 open 方法都接受 const char* 类型的文件路径，而不是string类型；因此需要使用 .c_str() 将 string类型 转换成为 const char* 类型
    fTimes.open(strPathTimes.c_str());
    
    //预分配两个向量的容量为 5000， 避免向元素中添加元素的时候 频繁的分配内存，以提高性能
    vTimeStamps.reserve(5000);
    vstrImages.reserve(5000);

    //输入函数
    //注意，这里的读取只使用了 strPathTimes 中的数据，将时间映射到图片上，从而读取图片。毕竟这里的时间和图片名字对应关系非常好
    while(!fTimes.eof())
    {
        string s;
        //利用 fTimes 从文件中读取一行到字符串 s
        getline(fTimes,s);
        //检查是否读取到非空行
        if(!s.empty())
        {
            stringstream ss;
            ss << s;
            vstrImages.push_back(strImagePath + "/" + ss.str() + ".png"); // 例子: /home/peiweipan/Data ... /data/xxxxxxxxxxxxx.png
            double t;
            ss >> t;
            vTimeStamps.push_back(t/1e9);

        }
    }
}

/**
 * @brief IMU数据加载
 *  
 * @param[in] strImuPath    已有的IMU文件路径
 * @param[in] vTimeStamps   待输入的IMU输入
 * @param[in] vAcc          待输入的加速度计参数
 * @param[in] vGyro         待输入的陀螺仪计参数
 */
void LoadIMU(const string &strImuPath, vector<double> &vTimeStamps, vector<cv::Point3f> &vAcc, vector<cv::Point3f> &vGyro)
{
    //创建输入流
    ifstream fImu;
    fImu.open(strImuPath.c_str());
    
    //预分配内存
    vTimeStamps.reserve(5000);
    vAcc.reserve(5000);
    vGyro.reserve(5000);

    while(!fImu.eof())
    {
        string s;
        getline(fImu,s);
        //注释行跳过
        if (s[0] == '#')
            continue;

        if(!s.empty())
        {
            string item;        //存储分割后的字符串项
            size_t pos = 0;     //记录分隔符的位置
            double data[7];     //存储解析后的数据
            int count = 0;      
            while ((pos = s.find(',')) != string::npos) {       //找逗号分隔符直到没有为止
                item = s.substr(0, pos);                       //提取逗号之前的字符串
                data[count++] = stod(item);                    //转换成double类型
                s.erase(0, pos + 1);                           //删除这个字符串以及后面的逗号
            }
            item = s.substr(0, pos);                           //最后一个也这么做
            data[6] = stod(item);

            vTimeStamps.push_back(data[0]/1e9);                     //时间 从 纳秒 转换为 秒
            vAcc.push_back(cv::Point3f(data[4],data[5],data[6]));   //加速度计 XYZ 添加到 vAcc
            vGyro.push_back(cv::Point3f(data[1],data[2],data[3]));  //陀螺仪计 XYZ 添加到 vGyro
        }
    }
}
