#include <iostream>
#include <thread>
#include <fstream>
#include <string>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/ndt.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/visualization/pcl_visualizer.h>

using namespace std::chrono_literals;
using namespace pcl;
using namespace std;

int main(int argc, char *argv[])
{

    for (int index = 1; index < 142; index++)
    {
        string source_path = "scan_" + std::to_string(index) + ".pcd";
        string target_path = "scan_" + std::to_string(index - 1) + ".pcd";
        string out_path = "transformation_" + std::to_string(index) + "_" + std::to_string(index - 1) + ".txt";
        
        pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        if (pcl::io::loadPCDFile<pcl::PointXYZ>(target_path, *target_cloud) == -1)
        {
            PCL_ERROR("Couldn't read file first(target)");
            return (-1);
        }
        pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        if (pcl::io::loadPCDFile<pcl::PointXYZ>(source_path, *input_cloud) == -1)
        {
            PCL_ERROR("Couldn't read file second(source)");
            return (-1);
        }
        cout << "========\nStart Proc. " << index << "-" << index - 1 << endl;
        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::ApproximateVoxelGrid<pcl::PointXYZ> approximate_voxel_filter;
        approximate_voxel_filter.setLeafSize(1, 1, 1);
        approximate_voxel_filter.setInputCloud(input_cloud);
        approximate_voxel_filter.filter(*filtered_cloud);

        // Initializing Normal Distributions Transform (NDT).
        pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt;

        // Setting scale dependent NDT parameters
        // Setting minimum transformation difference for termination condition.
        ndt.setTransformationEpsilon(0.00001);
        // Setting maximum step size for More-Thuente line search.
        ndt.setStepSize(0.05);
        // Setting Resolution of NDT grid structure (VoxelGridCovariance).
        ndt.setResolution(1.0);
        
        // Setting max number of registration iterations.
        ndt.setMaximumIterations(10000000);

        // Setting point cloud to be aligned.
        ndt.setInputSource(filtered_cloud); //input_cloud
        // Setting point cloud to be aligned to.
        //ndt.setInputSource(input_cloud);
        ndt.setInputTarget(target_cloud);

        // Set initial alignment estimate found using robot odometry.
        Eigen::AngleAxisf init_rotation(0, Eigen::Vector3f::UnitZ());
        Eigen::Translation3f init_translation(0, 0, 0);
        Eigen::Matrix4f init_guess = (init_translation * init_rotation).matrix();
        

        pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        ndt.align(*output_cloud, init_guess);

        std::cout << "Normal Distributions Transform has converged:" << ndt.hasConverged ()
             << " score: " << ndt.getFitnessScore () << std::endl<<ndt.getFinalTransformation()<<std::endl;
        fstream out_put_result;
        out_put_result.open(out_path, ios::out);
        out_put_result << ndt.getFinalTransformation();
        out_put_result.close();
    }
    return (0);
}