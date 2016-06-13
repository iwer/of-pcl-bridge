#include "of-pcl-bridge/of-pcl-bridge.h"
#define PCL_TO_OF_SCALE 1000

void toOfTexture(boost::shared_ptr<pcl::io::Image> image, ofTexture & texture)
{
	auto width = image->getWidth();
	auto height = image->getHeight();
	auto encoding = image->getEncoding();

	if (encoding == pcl::io::Image::Encoding::RGB)
	{
		auto data = static_cast<const unsigned char *>(image->getData());
		texture.loadData(data, width, height, GL_RGB);
	}
}

void toOfImage(boost::shared_ptr<pcl::io::Image> image, ofImage & texture)
{
	auto width = image->getWidth();
	auto height = image->getHeight();
	auto encoding = image->getEncoding();

	if (encoding == pcl::io::Image::Encoding::RGB)
	{
		auto data = static_cast<const unsigned char *>(image->getData());
		texture.setFromPixels(data, width, height, OF_IMAGE_COLOR);
	}
}

void createOfMeshFromPoints(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr inputCloud, ofMesh &targetMesh)
{
	targetMesh.clear();
	targetMesh.setMode(OF_PRIMITIVE_POINTS);
	for (auto &p : inputCloud->points) {
		targetMesh.addVertex(ofVec3f(p.x * PCL_TO_OF_SCALE, p.y * PCL_TO_OF_SCALE, p.z * PCL_TO_OF_SCALE));
		targetMesh.addColor(ofColor(p.r, p.g, p.b));
	}
}

void createOfMeshFromPoints(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr inputCloud, ofColor color, ofMesh &targetMesh)
{
	if (inputCloud) {
		// triangle inputMesh
		targetMesh.clear();
		targetMesh.setMode(OF_PRIMITIVE_POINTS);
		for (auto &p : inputCloud->points) {
			targetMesh.addVertex(ofVec3f(p.x * PCL_TO_OF_SCALE, p.y * PCL_TO_OF_SCALE, p.z * PCL_TO_OF_SCALE));
			targetMesh.addColor(color);
		}
	}
}

Eigen::Vector4f toEigenVector4f(ofVec3f& ofVec)
{
	Eigen::Vector4f ret;
	toEigenVector4f(ofVec, ret);
	return ret;
}

Eigen::Quaternionf toEigenQuaternionf(ofQuaternion& ofQuat)
{
	Eigen::Quaternionf ret;
	toEigenQuaternionf(ofQuat, ret);
	return ret;
}

ofVec3f toOfVector3(Eigen::Vector4f& pclVec)
{
	ofVec3f ret;
	toOfVector3(pclVec, ret);
	return ret;
}

ofQuaternion toOfQuaternion(Eigen::Quaternionf& pclQuat)
{
	ofQuaternion ret;
	toOfQuaternion(pclQuat, ret);
	return  ret;
}

ofMatrix4x4 toOfMatrix4x4(Eigen::Affine3f& pclMat)
{
	ofMatrix4x4 mat;
	
	mat.set(pclMat(0, 0), pclMat(0, 1), pclMat(0, 2), pclMat(0, 3),
		pclMat(1, 0), pclMat(1, 1), pclMat(1, 2), pclMat(1, 3),
		pclMat(2, 0), pclMat(2, 1), pclMat(2, 2), pclMat(2, 3),
		pclMat(3, 0), pclMat(3, 1), pclMat(3, 2), pclMat(3, 3));

	return mat;
}

void toEigenVector4f(ofVec3f &ofVec, Eigen::Vector4f &pclVec)
{
	pclVec.x() = ofVec.x / PCL_TO_OF_SCALE;
	pclVec.y() = ofVec.y / PCL_TO_OF_SCALE;
	pclVec.z() = ofVec.z / PCL_TO_OF_SCALE;
	pclVec.w() = 0;
}
void toEigenQuaternionf(ofQuaternion &ofQuat, Eigen::Quaternionf & pclQuat)
{
	pclQuat.w() = ofQuat.w();
	pclQuat.x() = ofQuat.x();
	pclQuat.y() = ofQuat.y();
	pclQuat.z() = ofQuat.z();
}

void toOfVector3(Eigen::Vector4f &pclVec, ofVec3f &ofVec)
{
	ofVec.set(pclVec.x() * PCL_TO_OF_SCALE, pclVec.y() * PCL_TO_OF_SCALE, pclVec.z() * PCL_TO_OF_SCALE);
}
void toOfQuaternion(Eigen::Quaternionf & pclQuat, ofQuaternion &ofQuat)
{
	ofQuat.set(pclQuat.x(), pclQuat.y(), pclQuat.z(), pclQuat.w());
}

void createOfMeshFromPointsAndTriangles(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr src, boost::shared_ptr<std::vector<pcl::Vertices>> triangles, ofMesh &targetMesh)
{
	if (triangles && src) {
		// triangle inputMesh
		targetMesh.clear();
		targetMesh.setMode(OF_PRIMITIVE_TRIANGLES);
		pcl::PointXYZRGB p;
		for (auto &t : *triangles) {
			for (auto &pointindex : t.vertices) {
				p = src->at(pointindex);
				ofVec3f ofp = ofVec3f(p.x * PCL_TO_OF_SCALE, p.y * PCL_TO_OF_SCALE, p.z * PCL_TO_OF_SCALE);
				targetMesh.addVertex(ofp);
				targetMesh.addColor(ofColor(p.r, p.g, p.b));
			}
		}
	}
}
