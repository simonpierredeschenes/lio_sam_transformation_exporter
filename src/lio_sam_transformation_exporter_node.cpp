#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf2_ros/transform_listener.h>
#include <pointmatcher/PointMatcher.h>
#include <pointmatcher_ros/PointMatcher_ROS.h>
#include <fstream>

typedef PointMatcher<float> PM;

std::unique_ptr<tf2_ros::Buffer> tfBuffer;
int nbRegistrations = 0;
PM::TransformationParameters firstPose;
PM::TransformationParameters lastPose;

PM::TransformationParameters rosMsgToPointMatcherPose(const geometry_msgs::Pose& pose)
{
	Eigen::Vector3f epsilon(pose.orientation.x, pose.orientation.y, pose.orientation.z);
	float eta = pose.orientation.w;
	PM::Matrix skewSymmetricEpsilon = PM::Matrix::Zero(3, 3);
	skewSymmetricEpsilon << 0, -epsilon[2], epsilon[1],
			epsilon[2], 0, -epsilon[0],
			-epsilon[1], epsilon[0], 0;
	PM::Matrix rotationMatrix = (((eta * eta) - epsilon.dot(epsilon)) * PM::Matrix::Identity(3, 3)) +
								(2 * eta * skewSymmetricEpsilon) + (2 * epsilon * epsilon.transpose());

	Eigen::Vector3f positionVector(pose.position.x, pose.position.y, pose.position.z);

	PM::TransformationParameters transformationParameters = PM::TransformationParameters::Identity(4, 4);
	transformationParameters.topLeftCorner(3, 3) = rotationMatrix.topLeftCorner(3, 3);
	transformationParameters.topRightCorner(3, 1) = positionVector.head(3);
	return transformationParameters;
}

PM::TransformationParameters findTransform(std::string sourceFrame, std::string targetFrame, ros::Time time, int transformDimension)
{
	geometry_msgs::TransformStamped tf = tfBuffer->lookupTransform(targetFrame, sourceFrame, time, ros::Duration(0.1));
	return PointMatcher_ROS::rosTfToPointMatcherTransformation<float>(tf, transformDimension);
}

void callback(const nav_msgs::Odometry& msg)
{
	try
	{
		PM::TransformationParameters robotToImu = findTransform("base_link", "mti30", msg.header.stamp, 4);
		PM::TransformationParameters currentPose = rosMsgToPointMatcherPose(msg.pose.pose) * robotToImu;
		if((++nbRegistrations) == 6)
		{
			firstPose = currentPose;
		}
		lastPose = currentPose;
	}
	catch(const tf2::TransformException& ex)
	{
		ROS_WARN("%s", ex.what());
		return;
	}
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "lio_sam_transformation_exporter_node");

	ros::NodeHandle nodeHandle;
	ros::NodeHandle privateNodeHandle;

	std::string finalTransformationFileName;
	privateNodeHandle.param<std::string>("final_transformation_file_name", finalTransformationFileName, "final_transformation.txt");

	tfBuffer = std::unique_ptr<tf2_ros::Buffer>(new tf2_ros::Buffer);
	tf2_ros::TransformListener tfListener(*tfBuffer);

	ros::Subscriber sub = nodeHandle.subscribe("/lio_sam/mapping/odometry", 10, callback);

	ros::spin();

	std::ofstream finalTransformationFile;
	finalTransformationFile.open(finalTransformationFileName, std::ios::app);
	finalTransformationFile << firstPose.inverse() * lastPose << std::endl;
	finalTransformationFile.close();

	return 0;
}
