#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <pcl/visualization/pcl_visualizer.h>
#include <vtkRenderWindow.h>


#include <QMainWindow>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <yaml-cpp/yaml.h>
#include <boost/shared_ptr.hpp>

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private:
    void _initUi();
    void _initYaml();
    void _initVtk();
    void _initConnect();
    Eigen::MatrixXd _parse_matrix(const YAML::Node &node);
    Eigen::VectorXd _parse_vector(const YAML::Node &node);
    Eigen::Affine3d _interpolatePose(double st,
                                     double et,
                                     const Eigen::Affine3d &s_pose,
                                     const Eigen::Affine3d &e_pose,
                                     double t);
    void _parse_odom(const std::string& odo_txt,
                     std::vector<double>& times,
                     std::vector<Eigen::Affine3d>& poses);
    void _parse_pcd(const std::string& pcd_dir,
                    std::vector<double>& times,
                    std::vector<std::string>& paths,
                    std::vector<bool>& b_intered,
                    std::vector<Eigen::Affine3d>& poses);
    void _inter_pcd_pose(const std::vector<double>& pcd_times,
                         const std::vector<double>& odom_times,
                         const std::vector<Eigen::Affine3d>& odom_poses,
                         std::vector<bool>& b_intered,
                         std::vector<Eigen::Affine3d>& pcd_poses);
    void _visualize_reset();
    void _visualize_reset_odom(std::string pre_name);
    void _visualize_odom(const std::vector<Eigen::Affine3d>& poses, double scale, std::string pre_name);
    void _visualize_reset_pcd();
    void _visualize_pcd(const std::vector<std::string>& pcd_paths, const std::vector<Eigen::Affine3d>& poses,double scale);
    void _visualize_show();

signals:
    void signalOdomChanged();
    void signalPcdChanged();


public slots:
    void slotOnOdomChanged();
    void slotOnPcdChanged();
    void slotOnPcdSelectedChanged();


private slots:
    void on_actionopen_pcd_triggered();

    void on_actionopen_odom_triggered();

    void on_actionopen_yaml_triggered();

private:
    Ui::MainWindow *ui;
    Eigen::MatrixXd lidar2odom_ext_;
    std::string pcd_dir_;
    std::string odo_txt_;

    std::vector<double> rtk_timestamps_;
    std::vector<Eigen::Affine3d> rtk_poses_;

    std::vector<std::string> lidar_paths_;
    std::vector<double> lidar_timestamps_;
    std::vector<bool> lidar_b_intered_;
    std::vector<Eigen::Affine3d> lidar_inter_poses_; // interpolate pose
//    std::vector<Eigen::Affine3d> lidar_odom_poses_; // only lidar odom pose TODO

    //
    std::vector<std::string> selected_paths_;
    std::vector<Eigen::Affine3d> selected_poses_;

    //
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer_ptr_;
    double scale_;
};
#endif // MAINWINDOW_H
