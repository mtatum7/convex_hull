#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/point_cloud_color_handlers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/keypoints/harris_3d.h>
#include <pcl/common/centroid.h>
#include <pcl/common/common.h>
#include <pcl/common/distances.h>
#include <pcl/common/intersections.h>
#include <pcl/common/centroid.h>
#include <pcl/common/geometry.h>
#include <pcl/features/organized_edge_detection.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>

#include <pcl/filters/passthrough.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/surface/concave_hull.h>
#include <fstream>
#include <math.h>
#include <stdlib.h>
#include <string> 
#include <vector> 
#define PI  3.14159

using namespace Eigen;

// PCL_EXPORTS template <typename Scalar> bool

struct CustomCompare
{
    bool operator()(const pcl::PointXYZ pt1, const pcl::PointXYZ pt2)
    {
        return (pt1.x < pt2.x || pt1.y < pt2.y);
    }
};

bool equalPoint(pcl::PointXYZ p1, pcl::PointXYZ p2){
    if (floor(p1.x) == floor(p2.x) && floor(p1.y) == floor(p2.y) && floor(p1.z) == floor(p2.z))
        return true;
    return false;
}

int main (int argc, char** argv)
{
    // Read in the cloud data
    pcl::PCDReader reader;
    pcl::PCLPointCloud2::Ptr cloud2 (new pcl::PCLPointCloud2);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>), cloud_f (new pcl::PointCloud<pcl::PointXYZ>), mod_cloud (new pcl::PointCloud<pcl::PointXYZ>), cloud_projected (new pcl::PointCloud<pcl::PointXYZ>);
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloud_vec;
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normal(new pcl::PointCloud<pcl::Normal>);
    std::vector<pcl::PointCloud<pcl::Normal>::Ptr> cloud_normal_vec;
    pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal>nor;
    reader.read ("hawkins.pcd", *cloud);

    for(int i = 0; i < cloud->points.size(); i++) {
        pcl::PointXYZ point;
        point.x = cloud->points[i].x;
        point.y = cloud->points[i].y;
        if(cloud->points[i].z>atof(argv[1]) && cloud->points[i].z<atof(argv[2])) {
            point.z = 0;
            mod_cloud->points.push_back(point);
        }
    }
    cloud = mod_cloud;
    // pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(cloud, 255, 255, 255);
    // viewer->addPointCloud(cloud, single_color, "point_cloud_og");

    // Build a filter to remove spurious NaNs
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud (cloud);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (0, 1.1);
    pass.filter (*cloud_f);
    std::cerr << "PointCloud after filtering has: " << cloud_f->points.size () << " data points." << std::endl;
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(cloud_f, 255, 255, 255);
    viewer->addPointCloud(cloud_f, single_color, "point_cloud_og");

    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    // Optional
    seg.setOptimizeCoefficients (true);
    // Mandatory
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setDistanceThreshold (atof(argv[3])); //0.001

    seg.setInputCloud (cloud_f);
    seg.segment (*inliers, *coefficients);

    // Project the model inliers
    pcl::ProjectInliers<pcl::PointXYZ> proj;
    proj.setModelType (pcl::SACMODEL_PLANE);
    proj.setInputCloud (cloud_f);
    proj.setIndices (inliers);
    proj.setModelCoefficients (coefficients);
    proj.filter (*cloud_projected);

     // Create a Convex Hull representation of the projected inliers
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_hull (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::ConvexHull<pcl::PointXYZ> chull;
    chull.setInputCloud (cloud_projected);
    chull.reconstruct (*cloud_hull);

    pcl::PointCloud<pcl::PointXYZ>::Ptr nodup_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    std::vector<pcl::PointXYZ> nodup_ordered_vect;
    // std::set<pcl::PointXYZ> pt_set_compare;
    // for(int i = 0; i < cloud_hull->points.size(); i++) {
    //     if(pt_set_compare.find(cloud_hull->points[i]) == pt_set_compare.end()) {
    //         nodup_ordered_vect.push_back(cloud_hull->points[i]);
    //         nodup_cloud->points.push_back(cloud_hull->points[i]);
    //         pt_set_compare.insert(cloud_hull->points[i]);
    //     }
    // }

    // // custom compare set
    // std::set<pcl::PointXYZ, CustomCompare> pt_set_compare;
    // for(int i = 0; i < cloud_hull->points.size(); i++) {
    //     pt_set_compare.insert(cloud_hull->points[i]);
    // }
    // std::set<pcl::PointXYZ, CustomCompare>::iterator it = pt_set_compare.begin();
    // while(it != pt_set_compare.end()) {
    //     nodup_ordered_vect.push_back(*it);
    //     nodup_cloud->points.push_back(*it);
    //     it++;
    // }

    // unique 2
    auto unique_end = std::unique(cloud_hull->points.begin(), cloud_hull->points.end(), equalPoint);
    cloud_hull->points.erase(unique_end, cloud_hull->points.end());

    // // unique 
    // std::vector<pcl::PointXYZ>::iterator it;
    // it = std::unique (cloud_hull->points.begin(), cloud_hull->points.end());  
    // cloud_hull->points.resize( std::distance(cloud_hull->points.begin(),it) );
    // Voxels to prevent duplicates
    // pcl::VoxelGrid<pcl::PointXYZ> sor;
    // sor.setInputCloud(cloud_hull);
    // sor.setLeafSize(2.0f,2.0f,2.0f);
    // sor.filter(*cloud_hull);
    // chull.setInputCloud (cloud_hull);
    // chull.reconstruct (*cloud_hull);

    //Move to positive
    // float minx = 0.0;
    // float miny = 0.0;
    // for(int i = 0; i < cloud_hull->points.size (); i++) {
    //     if (cloud_hull->points[i].x < minx) {
    //         minx = cloud_hull->points[i].x;
    //     }
    //     if (cloud_hull->points[i].y < miny) {
    //         miny = cloud_hull->points[i].y;
    //     }
    // }
    // for(int i = 0; i < cloud_hull->points.size (); i++) {
    //     cloud_hull->points[i].x = cloud_hull->points[i].x - minx;
    //     cloud_hull->points[i].y = cloud_hull->points[i].y - miny;
    // }

    // std::cerr << "Final cloud has: " << nodup_cloud->points.size() << " data points." << std::endl;
    // pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color_ch(nodup_cloud, 255, 0, 0);
    // viewer->addPointCloud(nodup_cloud, single_color_ch, "point_cloud_ch");
    // viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "point_cloud_ch");
    // std::ofstream convex_file;
    // convex_file.open("nodup_cloud.pol", std::ofstream::out | std::ofstream::trunc); // append instead of overwrite
    // convex_file << nodup_cloud->points.size();
    // for(int i = 0; i < nodup_cloud->points.size(); i++) {
    //     // convex_file << " " << (int) floor(cloud_hull->points[i].x) << "/1 " << (int) floor(cloud_hull->points[i].y) << "/1";
    //     convex_file << " " << (int) floor(nodup_cloud->points[i].x) << ".0 " << (int) floor(nodup_cloud->points[i].y) << ".0";
    // }
    // convex_file.close();

    std::cerr << "Final cloud has: " << cloud_hull->points.size() << " data points." << std::endl;
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color_ch(cloud_hull, 255, 0, 0);
    viewer->addPointCloud(cloud_hull, single_color_ch, "point_cloud_ch");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "point_cloud_ch");
    std::ofstream convex_file;
    convex_file.open("cloud_hull.pol", std::ofstream::out | std::ofstream::trunc); // append instead of overwrite
    convex_file << cloud_hull->points.size();
    for(int i = 0; i < cloud_hull->points.size(); i++) {
        // convex_file << " " << (int) floor(cloud_hull->points[i].x) << "/1 " << (int) floor(cloud_hull->points[i].y) << "/1";
        convex_file << " " << (int) floor(cloud_hull->points[i].x) << ".0 " << (int) floor(cloud_hull->points[i].y) << ".0";
    }
    convex_file.close();


    //  // Create a Concave Hull representation of the projected inliers
    // pcl::PointCloud<pcl::PointXYZ>::Ptr cave_hull (new pcl::PointCloud<pcl::PointXYZ>);
    // pcl::ConcaveHull<pcl::PointXYZ> cavehull;
    // cavehull.setInputCloud (cloud_projected);
    // cavehull.setAlpha (atof(argv[4])); //0.1
    // cavehull.reconstruct (*cave_hull);
    // std::cerr << "Concave hull has: " << cave_hull->points.size () << " data points." << std::endl;
    // pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color_cave(cave_hull, 0, 255, 0);
    // viewer->addPointCloud(cave_hull, single_color_cave, "point_cloud_cave");
    // viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "point_cloud_cave");

    // // pcl::io::savePCDFileASCII ("concave_pcd.pcd", *cave_hull);
    // std::ofstream outfile;
    // outfile.open("concave_pos2.pol", std::ofstream::out | std::ofstream::trunc); // append instead of overwrite
    // outfile << cave_hull->points.size();
    // for(int i = 0; i < cave_hull->points.size(); i++) {
    //     // outfile << " " << (int) abs(cave_hull->points[i].x) << "/1 " << (int) abs(cave_hull->points[i].y) << "/1";
    //     outfile << " " << (int) abs(cave_hull->points[i].x) << ".0 " << (int) abs(cave_hull->points[i].y) << ".0";
    // }
    // outfile.close();



    // std::cout << "DONE" << std::endl;
    while (!viewer->wasStopped())
        {
            viewer->spinOnce(100);
            boost::this_thread::sleep(boost::posix_time::microseconds(100000));
            // std::this_thread::sleep_for(100);
        }

    return (0);
}


    // std::cout << "PointCloud before filtering has: " << cloud->points.size () << " data points." << std::endl; //*

    // // Create the filtering object: downsample the dataset using a leaf size of 1cm
    // pcl::VoxelGrid<pcl::PointXYZ> vg;
    // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
    // //   vg.setInputCloud (cloud);
    // //   float leaf_size = atof(argv[4]);
    // //   vg.setLeafSize (leaf_size, leaf_size, leaf_size);
    // //   vg.filter (*cloud_filtered);
    // //   std::cout << "PointCloud after filtering has: " << cloud_filtered->points.size ()  << " data points." << std::endl; //*
    // //   pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color1(cloud, 255, 0, 0);
    // //   viewer->addPointCloud(cloud_filtered, single_color1, "point_cloud_filtered");
    // cloud_filtered = cloud;

    // // Create the segmentation object for the planar model and set all the parameters
    // pcl::SACSegmentation<pcl::PointXYZ> seg;
    // pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    // pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    // pcl::PointIndices::Ptr inliers2 (new pcl::PointIndices);
    // pcl::ModelCoefficients::Ptr coefficients2 (new pcl::ModelCoefficients);
    // std::vector<std::vector<float>> coeff_vec;
    // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZ> ());
    // //   pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
    // seg.setOptimizeCoefficients (true);
    // seg.setModelType (pcl::SACMODEL_LINE);
    // seg.setMethodType (pcl::SAC_RANSAC);
    // int max_iter = atoi(argv[5]);
    // seg.setMaxIterations (max_iter);
    // float dist_thresh = atof(argv[2]);
    // std::cout << "dist_thresh " << dist_thresh << std::endl;
    // seg.setDistanceThreshold (dist_thresh); //0.2 good;“distance threshold”, which determines how close a point must be to the model in order to be considered an inlier
    // int nr_points = (int) cloud_filtered->points.size ();
    // int j = 0;
    // std::vector<pcl::ModelCoefficients> lines_vec;
    // float point_frac = atof(argv[3]);
    // int line_num = 0;
    // int l = 0;
    // std::map<int,std::vector<int>> centroid_map;
    // std::vector<pcl::PointXYZ> centroid_vec;
    // while (cloud_filtered->points.size () > point_frac * nr_points)
    // {
    //     // Segment the largest planar component from the remaining cloud
    //     seg.setInputCloud (cloud_filtered);
    //     seg.segment (*inliers, *coefficients);
    //     coeff_vec.insert(coeff_vec.begin(),coefficients->values);
    //     if (inliers->indices.size () == 0)
    //     {
    //     std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
    //     break;
    //     }

    //     // Extract the planar inliers from the input cloud
    //     pcl::ExtractIndices<pcl::PointXYZ> extract;
    //     extract.setInputCloud (cloud_filtered);
    //     extract.setIndices (inliers);
    //     extract.setNegative (false);

    //     // Get the points associated with the planar surface
    //     extract.filter (*cloud_plane);
    //     std::cout << "PointCloud representing the planar component: " << cloud_plane->points.size() << " data points." << std::endl;
    //     cloud_vec.push_back(cloud_plane);

    //     // Remove the planar inliers, extract the rest
    //     extract.setNegative (true);
    //     extract.filter (*cloud_f);
    //     *cloud_filtered = *cloud_f;


    //     //CLUSTERING
    //     pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    //     tree->setInputCloud (cloud_plane);

    //     std::vector<pcl::PointIndices> cluster_indices;
    //     pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    //     ec.setClusterTolerance (atof(argv[7])); // 2cm If you take a very small value, it can happen that an actual object can be seen as multiple clusters. 
    //     //On the other hand, if you set the value too high, it could happen, that multiple objects are seen as one cluster. 
    //     ec.setMinClusterSize(atoi(argv[6])); //100
    //     ec.setMaxClusterSize (INT_MAX); //25000
    //     ec.setSearchMethod (tree);
    //     ec.setInputCloud (cloud_plane);
    //     ec.extract (cluster_indices);

    //     // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
    //     // std::vector<pcl::PointXYZ> centroid_vec;
    //     // int l = 0;
    //     for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
    //     {
    //         pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>());
    //         for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
    //             cloud_cluster->points.push_back (cloud_plane->points[*pit]); //*
    //         cloud_cluster->width = cloud_cluster->points.size ();
    //         cloud_cluster->height = 1;
    //         cloud_cluster->is_dense = true;

    //         std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;

    //         int r = rand()%255;
    //         int g = rand()%255;
    //         int b = rand()%255;
    //         pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(cloud_cluster, r, g, b);
    //         viewer->addPointCloud(cloud_cluster, single_color, "pc" + l);
    //         viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "pc" + l);
    //         viewer->addCoordinateSystem(1);
    //         pcl::CentroidPoint<pcl::PointXYZ> centroid;
    //         for(int i = 0; i < cloud_cluster->points.size(); i++) {
    //             centroid.add(cloud_cluster->points[i]);
    //         }
    //         pcl::PointXYZ c1;
    //         centroid.get (c1);
    //         centroid_vec.push_back(c1);
    //         viewer->addText3D(std::to_string(l), c1, 0.25, (double) r/255, (double) g/255, (double) b/255);
    //         pcl::ModelCoefficients line_coeff2;
    //         seg.setInputCloud (cloud_cluster);
    //         seg.segment (*inliers2, *coefficients2);
    //         for(int k = 0; k < coefficients2->values.size(); k++) {
    //             line_coeff2.values.push_back(coefficients2->values[k]);
    //         }
    //         lines_vec.push_back(line_coeff2);
    //         viewer->addLine(line_coeff2, std::string("line" + line_num));
    //         viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_SURFACE, "line" + line_num); 
    //         viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, (double) r/255, (double) g/255, (double) b/255, "line" + line_num);
    //         viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 10.0, "line" + line_num);
    //         viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 2, "line" + line_num);
    //         line_num++;
    //         l++;
    //     }
    //     j++;
    // }

    // for(int i = 0; i < centroid_vec.size(); i++) {
    //     pcl::PointXYZ p1 = centroid_vec.at(i);
    //     // float min_dist = Inf;
    //     for(int m = 1; m < centroid_vec.size(); m++) {
    //         if(i != m) {
    //             pcl::PointXYZ p2 = centroid_vec.at(m);
    //             float dist = pcl::geometry::distance(p1, p2);
    //             if(dist < atof(argv[8])) {
    //                 centroid_map[i].push_back(m);
    //             }
    //         }
    //     }
    // }
    // //CORNER DETECTION
    // int corner_num = 0;
    // for(int i = 0; i < centroid_map.size(); i++) {
    //     pcl::ModelCoefficients line1 = lines_vec.at(i);
    //     Vector4f v1 = Vector4f(line1.values[3], line1.values[4], line1.values[5], 0);
    //     std::vector<int> neighbor_clusters = centroid_map[i];
    //     for(int j = 0; j < neighbor_clusters.size(); j++) {
    //         pcl::ModelCoefficients line2 = lines_vec.at(neighbor_clusters.at(j));
    //         Vector4f v2 = Vector4f(line2.values[3], line2.values[4], line2.values[5], 0);
    //         line_num = 0;
    //         double angle = pcl::getAngle3D(v1,v2);
    //         // std::cout << "ANGLE " << angle << std::endl;
    //         Eigen::Vector4f point;
    //         double sqr_eps = 1e-4;
    //         bool not_parallel = pcl::lineWithLineIntersection (line1, line2, point, sqr_eps);
    //         std::cout << "Succeeded/Not Parallel? " << not_parallel <<std::endl;
    //         // std::cout << "line " << line << std::endl;
    //         // if((angle > PI/4 && angle < 3*PI/4) || (angle > 5*PI/4 && angle < 7*PI/4)) {
    //         if((angle > 0.5 && angle < (PI-0.5)) || (angle > (PI+0.5)) && (angle < 2*PI-0.5)) {
    //             std::cout << "CORNER between " << i << " & "<< neighbor_clusters.at(j) << std::endl;
    //             pcl::ModelCoefficients point_coeff;
    //             for(int k = 0; k < point.size(); k++) {
    //                 // std::cout << "line val " << line[i] << std::endl;
    //                 point_coeff.values.push_back(point[k]);
    //             }
    //             pcl::PointXYZ pp;
    //             pp.getVector4fMap() = point;
    //             viewer->addSphere(pp, 1, 0, 0.5, 0, std::string("corner" + corner_num));
    //             corner_num++;
    //         } else {
    //         std::cout << "NO corner between " << i << " & "<< neighbor_clusters.at(j) << std::endl;
    //         }
    //     }
    // }