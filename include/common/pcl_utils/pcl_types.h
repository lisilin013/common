// Copyright (c) 2020. All rights reserved.
// Author: lisilin013@163.com(Silin Li) on 20-9-1.

#pragma once
#include <stdint.h>

#include <Eigen/Dense>

#define PCL_NO_PRECOMPILE
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/pcl_base.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/search/organized.h>
#include <pcl/search/search.h>
#include <pcl/segmentation/extract_clusters.h>

/// \brief Custom PCL point type that can be extended with custom fields needed
/// by incremental algorithms. Custom points must be defined and registered in
/// the global namespace.
struct __PointExtended {
  inline __PointExtended(const __PointExtended &p)
      : data{p.x, p.y, p.z, 1.0f}, ed_cluster_id(p.ed_cluster_id), sc_cluster_id(p.sc_cluster_id) {}

  inline __PointExtended() : data{0.0f, 0.0f, 0.0f, 1.0f}, ed_cluster_id(0u), sc_cluster_id(0u) {}

  friend std::ostream &operator<<(std::ostream &os, const __PointExtended &p) {
    return os << "x: " << p.x << ", y: " << p.y << ", z: " << p.z
              << ", EuclideanDistance ID: " << p.ed_cluster_id
              << ", SmoothnessConstraints ID: " << p.sc_cluster_id;
  }

  // X, Y, Z components of the position of the point.
  // Memory layout (4 x 4 bytes): [ x, y, z, _ ]
  PCL_ADD_POINT4D;
  // Cluster ID fields.
  // Memory layout (4 x 4 bytes): [ ed_cluster_id, sc_cluster_id, _, _ ]
  union {
    struct {
      uint32_t ed_cluster_id; // Euclidean Distance ID
      uint32_t sc_cluster_id; // Smoothness Constraints ID
    };
    uint32_t data_c[4];
  };
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

// Register the point type.
POINT_CLOUD_REGISTER_POINT_STRUCT(
    __PointExtended,
    (float, x, x)(float, y, y)(float, z, z)(uint32_t, ed_cluster_id,
                                            ed_cluster_id)(uint32_t, sc_cluster_id, sc_cluster_id))

struct __PointXYZIR {
  PCL_ADD_POINT4D;                // quad-word XYZ
  float intensity;                ///< laser intensity reading
  uint16_t ring;                  ///< laser ring number
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW // ensure proper alignment
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(__PointXYZIR,
                                  (float, x, x)(float, y, y)(float, z, z)(float, intensity,
                                                                          intensity)(uint16_t, ring,
                                                                                     ring))

struct __PointXYZIRL {
  PCL_ADD_POINT4D; // quad-word XYZ
  float intensity; ///< laser intensity reading
  uint16_t ring;   ///< laser ring number
  uint16_t label;  ///< point semantic label

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW // ensure proper alignment
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(
    __PointXYZIRL, (float, x, x)(float, y, y)(float, z, z)(float, intensity,
                                                           intensity)(uint16_t, ring,
                                                                      ring)(uint16_t, label, label))

struct __PointXYZIRT {
  PCL_ADD_POINT4D; // quad-word XYZ
  float intensity; ///< laser intensity reading
  uint16_t ring;   ///< laser ring number
  double timestamp;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW // ensure proper alignment
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(__PointXYZIRT,
                                  (float, x, x)(float, y, y)(float, z, z)(float, intensity,
                                                                          intensity)(
                                      uint16_t, ring, ring)(double, timestamp, timestamp))

/*
 * A point cloud type that has 6D pose info ([x,y,z,roll,pitch,yaw] intensity is
 * time stamp)
 */
struct __PointXYZIRPYT {
  PCL_ADD_POINT4D
  PCL_ADD_INTENSITY; // preferred way of adding a XYZ+padding
  float roll;
  float pitch;
  float yaw;
  double time;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW // make sure our new allocators are aligned
} EIGEN_ALIGN16;                  // enforce SSE padding for correct memory alignment

POINT_CLOUD_REGISTER_POINT_STRUCT(__PointXYZIRPYT,
                                  (float, x, x)(float, y, y)(float, z, z)(float, intensity,
                                                                          intensity)(float, roll,
                                                                                     roll)(
                                      float, pitch, pitch)(float, yaw, yaw)(double, time, time))

namespace pcl_utils {

/*
 * \brief Type representing IDs, for example for segments or clouds
 * Warning: the current implementation sometimes uses IDs as array indices.
 */
typedef int64_t Id;
const Id kNoId = -1;
const Id kInvId = -2;
const Id kUnassignedId = -3; // Used internally by the incremental segmenter.

typedef __PointExtended MapPoint;
typedef __PointXYZIR PointIR;
typedef __PointXYZIRL PointIRL;
typedef __PointXYZIRT PointIRT;
typedef __PointXYZIRPYT PointXYZIRPYT;

typedef pcl::PointXYZ Point;
typedef pcl::PointXYZI PointI;
typedef pcl::PointXYZRGB PointRGB;
typedef pcl::PointXYZRGBL PointRGBL;

typedef pcl::PointCloud<Point> PointCloud;
typedef pcl::PointCloud<PointI> PointICloud;
typedef pcl::PointCloud<PointIR> PointIRCloud;
typedef pcl::PointCloud<PointIRT> PointIRTCloud;
typedef pcl::PointCloud<PointIRL> PointIRLCloud;
typedef pcl::PointCloud<PointRGB> PointRGBCloud;
typedef pcl::PointCloud<PointRGBL> PointRGBLCloud;
typedef pcl::PointCloud<PointXYZIRPYT> PointIRPYTCloud;

typedef PointCloud::Ptr PointCloudPtr;
typedef PointICloud::Ptr PointICloudPtr;

} // namespace pcl_utils
