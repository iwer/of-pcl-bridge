#pragma once

#include "ofMain.h"
#include "pcl/common/common.h"
#include <pcl/io/image.h>
#include <pcl/Vertices.h>


void toOfTexture(boost::shared_ptr<pcl::io::Image> image, ofTexture & texture);
void toOfImage(boost::shared_ptr<pcl::io::Image> image, ofImage & texture);

void createOfMeshFromPoints(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr inputCloud, ofMesh &targetMesh);
void createOfMeshFromPoints(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr inputCloud, ofColor color, ofMesh &targetMesh);
void createOfMeshFromPointsWNormals(pcl::PointCloud<pcl::PointXYZRGBNormal>::ConstPtr inputCloud, ofMesh &targetMesh);
void createOfMeshFromPointsAndTriangles(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr src, boost::shared_ptr<std::vector<pcl::Vertices>> triangles, ofMesh &targetMesh);
void createOfMeshFromPointsWNormalsAndTriangles(pcl::PointCloud<pcl::PointXYZRGBNormal>::ConstPtr src, boost::shared_ptr<std::vector<pcl::Vertices>> triangles, ofMesh &targetMesh);

void toEigenVector4f(ofVec3f &ofVec, Eigen::Vector4f &pclVec);
Eigen::Vector4f toEigenVector4f(ofVec3f &ofVec);
void toEigenQuaternionf(ofQuaternion &ofQuat, Eigen::Quaternionf & pclQuat);
Eigen::Quaternionf toEigenQuaternionf(ofQuaternion &ofQuat);

void toOfVector3(Eigen::Vector4f &pclVec, ofVec3f &ofVec);
ofVec3f toOfVector3(Eigen::Vector4f &pclVec);
void toOfQuaternion(Eigen::Quaternionf & pclQuat, ofQuaternion &ofQuat);
ofQuaternion toOfQuaternion(Eigen::Quaternionf & pclQuat);

ofMatrix4x4 toOfMatrix4x4(Eigen::Affine3f& pclMat);
