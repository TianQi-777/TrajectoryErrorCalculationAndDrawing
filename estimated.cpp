#include <Eigen/Core>
#include <Eigen/Geometry>
#include <sophus/se3.h>
#include <string>
#include <iostream>
#include <fstream>
#include <cmath>
#include <pangolin/pangolin.h>

using namespace std;

void DrawTrajectory(vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>>,vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>>);
                                                                            //一定不要忘了出入两个参数时，函数声明也应该是两个参数
int main(int argc, char **argv)
{

    vector<Sophus::SE3,Eigen::aligned_allocator<Sophus::SE3>> poses1;
    vector<Sophus::SE3,Eigen::aligned_allocator<Sophus::SE3>> poses2;
    vector<Sophus::SE3,Eigen::aligned_allocator<Sophus::SE3>> poses1_original;  //为了后面生成轨迹图形进行比较，不能用poses1，它代表逆

    ifstream fin1("../groundtruth.txt");
    ifstream fin2("../estimated.txt");

    int i=0;

    while (fin1)  //读取真实轨迹参数
    {
        double data1[8] = {0};  //其中data1[0]表示时间，跳过（略去）
        for (auto &d1:data1)
            fin1 >> d1;

        Eigen::Vector3d t1(data1[1], data1[2], data1[3]);    //获取平移数据

        Eigen::Quaterniond q1(data1[7], data1[4], data1[5], data1[6]);    //获取旋转数据，注意输入是实1,虚1,虚2,虚3

        Sophus::SE3 T1(q1, t1);  //获取李群一组（行）数据的SE3

        Sophus::SE3 T1_temp = T1.inverse();  //为满足公式要求，inverse()取逆

        poses1.push_back(T1_temp);      //将取逆后的SE3存入poses1
        poses1_original.push_back(T1);  //将真实轨迹原SE3存入poses1_original

        i++;
    }

    while (fin2)   //将估计轨迹SE3存入poses2,方法同上
    {
        double data2[8] = {0};
        for (auto &d2:data2)
            fin2 >> d2;


        Eigen::Vector3d t2(data2[1], data2[2], data2[3]);

        Eigen::Quaterniond q2(data2[7], data2[4], data2[5], data2[6]);

        Sophus::SE3 T2(q2, t2);

        poses2.push_back(T2);


    }
    double sum = 0,RMSE=0,e_2=0;

    for(int n=0;n<i-1;n++)
    {

        Sophus::SE3 Temp1=poses1[n];
        Sophus::SE3 Temp2=poses2[n];

        Sophus::SE3 temp = Temp1.operator*(Temp2);  //令对应组的李群进行乘法（李群对乘法封闭，加法不封闭）
        //cout<<"输出相乘后的李群"<<temp.matrix()<<endl;
        Eigen::VectorXd se3=temp.log();     //进行对数变换
        ////cout<<"输出李代数"<<se3<<endl;
        e_2=se3.squaredNorm();   //计算每组e^2(取二范数）
        //cout<<"第n组数据误差*"<<e_2<<endl;
        sum+=e_2;   //求和
    }
    RMSE= sqrt(sum/i);  //对sum/i开根号得RMSE
    //cout<<"\nsum"<<sum<<endl;
    cout<<"\nRMSE"<<RMSE<<endl;
    poses1.pop_back();  //删除最后一个0元素
    poses1_original.pop_back();
    poses2.pop_back();

    DrawTrajectory(poses1_original,poses2); //读入两组数据对比轨迹差异
    return 0;
}

/*******************************************************************************************/
void DrawTrajectory(vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>> poses1,
                    vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>> poses2) {
    if (poses1.empty()&&poses2.empty()) {
        cerr << "Trajectory is empty!" << endl;
        return;
    }

    // create pangolin window and plot the trajectory
    pangolin::CreateWindowAndBind("Trajectory Viewer", 1024, 768);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    pangolin::OpenGlRenderState s_cam(
            pangolin::ProjectionMatrix(1024, 768, 500, 500, 512, 389, 0.1, 1000),
            pangolin::ModelViewLookAt(0, -0.1, -1.8, 0, 0, 0, 0.0, -1.0, 0.0)
    );

    pangolin::View &d_cam = pangolin::CreateDisplay()
            .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -1024.0f / 768.0f)
            .SetHandler(new pangolin::Handler3D(s_cam));


    while (pangolin::ShouldQuit() == false) {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        d_cam.Activate(s_cam);
        glClearColor(1.0f, 1.0f, 1.0f, 1.0f);

        glLineWidth(2);
        for (size_t i = 0; i < poses1.size() - 1; i++) {    //因为没有构成回环，这里减2更好，不然会连成一条直线，若为回环就-1,也可以用pop_back()
            glColor3f(1 - (float) i / poses1.size(), 0.0f, (float) i / poses1.size());
            glBegin(GL_LINES);
            auto p1 = poses1[i], p2 = poses1[i + 1];
            glVertex3d(p1.translation()[0], p1.translation()[1], p1.translation()[2]);
            glVertex3d(p2.translation()[0], p2.translation()[1], p2.translation()[2]);
            glEnd();
        }
        for (size_t i = 0; i < poses2.size() - 1; i++) {
            glColor3f(1 - (float) i / poses2.size(), 0.0f, (float) i / poses2.size());
            glBegin(GL_LINES);
            auto p1 = poses2[i], p2 = poses2[i + 1];
            glVertex3d(p1.translation()[0], p1.translation()[1], p1.translation()[2]);
            glVertex3d(p2.translation()[0], p2.translation()[1], p2.translation()[2]);
            glEnd();
        }
        pangolin::FinishFrame();
        usleep(5000);   // sleep 5 ms
    }

}