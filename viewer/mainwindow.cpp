#include "mainwindow.h"
#include "./ui_mainwindow.h"
#include <QDebug>

#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>

#include <QFileDialog>

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    //
    _initUi();
    _initVtk();
    _initConnect();

   //
    _initYaml();
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::_initUi()
{
    ui->splitter->setStretchFactor(0, 3);
    ui->splitter->setStretchFactor(1, 20);
    ui->checktreeview->setFilterList("*.pcd");
}

void MainWindow::_initYaml()
{
    QDir current_dir = QDir::current();
    QString current_dir_path = current_dir.path();
    QString yaml_file = current_dir_path + "/config.yaml";
    YAML::Node root = YAML::LoadFile(yaml_file.toStdString());

    lidar2odom_ext_ = _parse_matrix(root["lidar2odo"]);
    pcd_dir_ = root["pcd_dir"].as<std::string>();
    odo_txt_ = root["odom_txt"].as<std::string>();
    std::cout << lidar2odom_ext_ << std::endl;
    qDebug() << "pcd dir:" << QString::fromStdString(pcd_dir_);
    qDebug() << "odom txt:" << QString::fromStdString(odo_txt_);

    emit signalOdomChanged();
    emit signalPcdChanged();
}

void MainWindow::_initVtk()
{
    viewer_ptr_.reset(new pcl::visualization::PCLVisualizer("viewer", false));
    viewer_ptr_->addCoordinateSystem();
    ui->qvtkWidget->SetRenderWindow(viewer_ptr_->getRenderWindow());
    viewer_ptr_->setupInteractor(ui->qvtkWidget->GetInteractor(), ui->qvtkWidget->GetRenderWindow());
    ui->qvtkWidget->update();
}

void MainWindow::_initConnect()
{
    connect(this, &MainWindow::signalOdomChanged, this, &MainWindow::slotOnOdomChanged);
    connect(this, &MainWindow::signalPcdChanged, this, &MainWindow::slotOnPcdChanged);
    connect(ui->checktreeview, &CheckTreeView::selectedChanged, this, &MainWindow::slotOnPcdSelectedChanged);
}

Eigen::MatrixXd MainWindow::_parse_matrix(const YAML::Node &node)
{
    int rows = node.size();
    int cols = node.begin()->size();
    Eigen::MatrixXd mat;
    mat.resize(rows, cols);
    for (int i = 0; i < rows; i++)
        mat.row(i) = _parse_vector(node[i]);

    return mat;
}

Eigen::VectorXd MainWindow::_parse_vector(const YAML::Node &node)
{
    int nums = node.size();
    Eigen::VectorXd vec(nums);
    for (int i = 0; i < nums; i++)
        vec(i) = node[i].as<double>();

    return vec;
}

Eigen::Affine3d MainWindow::_interpolatePose(double st, double et, const Eigen::Affine3d &s_pose, const Eigen::Affine3d &e_pose, double t)
{
    // scale
    double ratio = (t - st) / (et - st);

    // interpolate
    Eigen::Quaterniond s_quat(s_pose.rotation());
    Eigen::Quaterniond e_quat(e_pose.rotation());
    Eigen::Quaterniond quat = s_quat.slerp(ratio, e_quat);
    Eigen::Vector3d trans = s_pose.translation() * (1.0 - ratio) + e_pose.translation() * ratio;

    // orgnize
    Eigen::Affine3d pose;
    pose.translation() = trans;
    pose.linear() = quat.toRotationMatrix();

    return pose;
}

void MainWindow::_parse_odom(const std::string &odo_txt, std::vector<double> &times, std::vector<Eigen::Affine3d> &poses)
{
    //
    int id;
    double timestamp;
    Eigen::Vector3d xyz_pre;
    Eigen::Vector3d xyz;
    Eigen::Quaterniond q;

    std::ifstream inf;
    inf.open(odo_txt);
    while (inf >> id >> timestamp >> xyz[0] >> xyz[1] >> xyz[2] >> q.x() >> q.y() >> q.z() >> q.w()) {
        //
        Eigen::Affine3d transform = Eigen::Affine3d::Identity();
        transform.prerotate(q);
        transform.pretranslate(xyz);
        poses.push_back(transform);
        times.push_back(timestamp);
    }
    inf.close();
}

void MainWindow::_parse_pcd(const std::string &pcd_dir,
                            std::vector<double>& times,
                            std::vector<std::string>& paths,
                            std::vector<bool>& b_intered,
                            std::vector<Eigen::Affine3d>& poses)
{
    //
    QDir dir(QString::fromStdString(pcd_dir));
    QFileInfoList files = dir.entryInfoList(QStringList("*.pcd"), QDir::Files, QDir::Name);
    foreach(const QFileInfo& file_info, files){
        paths.push_back(file_info.absoluteFilePath().toStdString());
        times.push_back(file_info.completeBaseName().toDouble());
        b_intered.push_back(false);
        poses.push_back(Eigen::Affine3d::Identity());
    }
}

void MainWindow::_inter_pcd_pose(const std::vector<double> &pcd_times,
                                 const std::vector<double> &odom_times,
                                 const std::vector<Eigen::Affine3d> &odom_poses,
                                 std::vector<bool>& b_intered,
                                 std::vector<Eigen::Affine3d> &pcd_poses)
{
    //
    pcd_poses.clear();
    //
    if (odom_times.size()<=1)
        return;
    //
    int i=0,j=0;
    while (j < pcd_times.size()) {
        const double& lidar_time = pcd_times[j];

        bool bfind = false;
        if(i+1 < odom_times.size()){
            if(odom_times[i]<=lidar_time && lidar_time<=odom_times[i+1]){
                bfind = true;
            }
        }

        if(bfind){
            double st = odom_times[i];
            double et = odom_times[i+1];
            Eigen::Affine3d s_pose = odom_poses[i];
            Eigen::Affine3d e_pose = odom_poses[i+1];

            b_intered[j] = true;
            pcd_poses[j] = _interpolatePose(st,et,s_pose,e_pose,lidar_time);
        }
        else{
            b_intered[j] = false;
            pcd_poses[j] = Eigen::Affine3d::Identity();
        }

        //
        if(bfind){
            if(i+1<odom_times.size())
                i++;
            j++;
        }
        else{
            if(lidar_time < odom_times[i])
                j++;
            else if(odom_times[i] < lidar_time)
                if(i+1<odom_times.size())
                    i++;
        }
    }
}

void MainWindow::_visualize_reset()
{
    //
    viewer_ptr_->removeAllPointClouds();
    viewer_ptr_->removeAllShapes();
    viewer_ptr_->removeAllCoordinateSystems();
}

void MainWindow::_visualize_reset_odom(std::string pre_name)
{
//    for(int i=0;i<poses.size();i++){
//        std::string name = pre_name + "_point_" + std::to_string(i);
//        viewer_ptr_->removeShape(name);
////        std::string coordinate_name = pre_name + "_cor_" + std::to_string(i);
////        viewer_ptr_->removeCoordinateSystem(coordinate_name);
//    }
    viewer_ptr_->removePointCloud(pre_name);
}

void MainWindow::_visualize_odom(const std::vector<Eigen::Affine3d> &poses, double scale,std::string pre_name)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr odom(new pcl::PointCloud<pcl::PointXYZ>());
    for(int i=0;i<poses.size();i++)
    {
        const auto &transform = poses[i];
        pcl::PointXYZ pt(transform.translation()(0), transform.translation()(1), transform.translation()(2));
        odom->points.push_back(pt);
    }
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_color(odom, 0, 255, 0);
    viewer_ptr_->addPointCloud(odom, cloud_color, pre_name);

//    //
//    for(int i=0;i<poses.size();i++){
//        const auto &transform = poses[i];
//        pcl::PointXYZ pt2(transform.translation()(0), transform.translation()(1), transform.translation()(2));
//        std::string name = pre_name + "_point_" + std::to_string(i);
//        viewer_ptr_->addSphere(pt2, 0.05*scale, 0.0, 1.0, 0.0, name);
////        //
////        std::string coordinate_name = pre_name + "_cor_" + std::to_string(i);
////        viewer_ptr_->addCoordinateSystem(0.2*scale, transform.cast<float>(), coordinate_name);
//    }
}

void MainWindow::_visualize_reset_pcd()
{
    std::string cloud_name = "cloud";
    viewer_ptr_->removePointCloud(cloud_name);
}

void MainWindow::_visualize_pcd(const std::vector<std::string> &pcd_paths, const std::vector<Eigen::Affine3d> &poses, double scale)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr all_clouds(new pcl::PointCloud<pcl::PointXYZ>());
    for(int i=0;i<pcd_paths.size();i++){
        Eigen::Matrix4f transform = poses[i].matrix().cast<float>()*lidar2odom_ext_.cast<float>();
//        std::cout << poses[i].matrix().cast<float>() << std::endl;
//        std::cout << transform << std::endl;

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::io::loadPCDFile(pcd_paths[i], *cloud);
        pcl::transformPointCloud(*cloud, *cloud, transform);
        *all_clouds += *cloud;
    }

    std::string name = "cloud";
    pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZ> fildColor(all_clouds, "z");
    viewer_ptr_->addPointCloud<pcl::PointXYZ>(all_clouds, fildColor, name);
    viewer_ptr_->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1*scale, name);
}

void MainWindow::_visualize_show()
{
//    viewer_ptr_->initCameraParameters();
//    viewer_ptr_->resetCamera();
    ui->qvtkWidget->update();
}

void MainWindow::slotOnOdomChanged(){
    std::string pre_name = "odom";
    //
    _visualize_reset_odom(pre_name);
    rtk_timestamps_.clear();
    rtk_poses_.clear();
    //
    _parse_odom(odo_txt_, rtk_timestamps_, rtk_poses_);
    //
    _visualize_odom(rtk_poses_, scale_, pre_name);
    _visualize_show();
}

void MainWindow::slotOnPcdChanged(){
    lidar_timestamps_.clear();
    lidar_paths_.clear();
    lidar_b_intered_.clear();
    lidar_inter_poses_.clear();
    //
    _parse_pcd(pcd_dir_, lidar_timestamps_, lidar_paths_, lidar_b_intered_, lidar_inter_poses_);
    _inter_pcd_pose(lidar_timestamps_, rtk_timestamps_, rtk_poses_, lidar_b_intered_, lidar_inter_poses_);

    ui->checktreeview->setFolderPath(QString::fromStdString(pcd_dir_));
}

void MainWindow::slotOnPcdSelectedChanged()
{
    //
    _visualize_reset_pcd();
    selected_paths_.clear();
    selected_poses_.clear();

    //
    QVector<int> selected_rows = ui->checktreeview->selectedRows();
    foreach(int i , selected_rows){
        selected_paths_.push_back(lidar_paths_[i]);
        selected_poses_.push_back(lidar_inter_poses_[i]);
    }

    //
    _visualize_pcd(selected_paths_, selected_poses_, scale_);
    _visualize_show();
}


void MainWindow::on_actionopen_pcd_triggered()
{
    QString selectedDir = QFileDialog::getExistingDirectory(
            this, tr("Select Pcd Dir"), QDir::homePath(),
            QFileDialog::ShowDirsOnly | QFileDialog::DontResolveSymlinks);
    if (!selectedDir.isEmpty())
    {
        pcd_dir_ = selectedDir.toStdString();
        emit signalPcdChanged();
    }
}


void MainWindow::on_actionopen_odom_triggered()
{

    QString fileName = QFileDialog::getOpenFileName(this, tr("Select Odom Txt"),
                                                    QDir::homePath(),
                                                    tr("Txt (*.txt)"));
    if(!fileName.isEmpty()){
        odo_txt_ = fileName.toStdString();
        emit signalOdomChanged();
    }


}


void MainWindow::on_actionopen_yaml_triggered()
{
    QString fileName = QFileDialog::getOpenFileName(this, tr("Select Yaml file"),
                                                    QDir::homePath(),
                                                    tr("Yaml (*.yaml *.yml)"));
    if(!fileName.isEmpty()){
        YAML::Node root = YAML::LoadFile(fileName.toStdString());

        lidar2odom_ext_ = _parse_matrix(root["lidar2odo"]);
        pcd_dir_ = root["pcd_dir"].as<std::string>();
        odo_txt_ = root["odom_txt"].as<std::string>();
        qDebug() << "pcd dir:" << QString::fromStdString(pcd_dir_);
        qDebug() << "odom txt:" << QString::fromStdString(odo_txt_);

        emit signalOdomChanged();
        emit signalPcdChanged();
    }
}

