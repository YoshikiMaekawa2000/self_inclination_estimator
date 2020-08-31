#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <tf/tf.h>
#include <Eigen/Core>
#include <Eigen/LU>

class ImuEKFRPY{
	private:
		/*node handle*/
		ros::NodeHandle _nh;
		ros::NodeHandle _nhPrivate;
		/*subscriber*/
		ros::Subscriber _sub_inipose;
		ros::Subscriber _sub_imu;
		ros::Subscriber _sub_bias;
		/*publisher*/
		ros::Publisher _pub_quat;
		/*state*/
		Eigen::Vector3d _x;	//(roll, pitch, yaw)
		Eigen::Matrix3d _P;
		/*bias*/
		sensor_msgs::Imu _bias;
		/*flag*/
		bool _got_inipose = false;
		bool _got_first_imu = false;
		bool _got_bias = false;
		/*parameter*/
		bool _wait_inipose;
		bool _use_quaternion_for_rotation;
		std::string _frame_id;
		double _sigma_ini;
		double _sigma_pre;
		double _sigma_obs;
		/*test*/
		sensor_msgs::Imu _imu_last;

	public:
		ImuEKFRPY();
		void initializeState(void);
		void callbackIniPose(const geometry_msgs::QuaternionStampedConstPtr& msg);
		void callbackIMU(const sensor_msgs::ImuConstPtr& msg);
		void callbackBias(const sensor_msgs::ImuConstPtr& msg);
		void predictionIMU(sensor_msgs::Imu imu, double dt);
		void observationIMU(sensor_msgs::Imu g);
		void publication(ros::Time stamp);
		void getRotMatrixRPY(double r, double p, Eigen::MatrixXd& Rot);
		void anglePiToPi(double& angle);
};

ImuEKFRPY::ImuEKFRPY()
	: _nhPrivate("~")
{
	std::cout << "--- imu_ekf_rpy ---" << std::endl;
	/*parameter*/
	_nhPrivate.param("wait_inipose", _wait_inipose, false);
	std::cout << "_wait_inipose = " << (bool)_wait_inipose << std::endl;
	_nhPrivate.param("use_quaternion_for_rotation", _use_quaternion_for_rotation, true);
	std::cout << "_use_quaternion_for_rotation = " << (bool)_use_quaternion_for_rotation << std::endl;
	_nhPrivate.param("frame_id", _frame_id, std::string("/base_link"));
	std::cout << "_frame_id = " << _frame_id << std::endl;
	_nhPrivate.param("sigma_ini", _sigma_ini, 1.0e-10);
	std::cout << "_sigma_ini = " << _sigma_ini << std::endl;
	_nhPrivate.param("sigma_pre", _sigma_pre, 1.0e-4);
	std::cout << "_sigma_pre = " << _sigma_pre << std::endl;
	_nhPrivate.param("sigma_obs", _sigma_obs, 1.0e+0);
	std::cout << "_sigma_obs = " << _sigma_obs << std::endl;
	/*sub*/
	_sub_inipose = _nh.subscribe("/initial_orientation", 1, &ImuEKFRPY::callbackIniPose, this);
	_sub_imu = _nh.subscribe("/imu/data", 1, &ImuEKFRPY::callbackIMU, this);
	_sub_bias = _nh.subscribe("/imu/bias", 1, &ImuEKFRPY::callbackBias, this);
	/*pub*/
	_pub_quat = _nh.advertise<geometry_msgs::QuaternionStamped>("/ekf/quat_rpy", 1);
	/*initialize*/
	initializeState();
}

void ImuEKFRPY::initializeState(void)
{
	_x = Eigen::Vector3d::Zero();
	_P = _sigma_ini*Eigen::Matrix3d::Identity();

	if(!_wait_inipose)	_got_inipose = true;

	std::cout << "_x = " << std::endl << _x << std::endl;
	std::cout << "_P = " << std::endl << _P << std::endl;
}

void ImuEKFRPY::callbackIniPose(const geometry_msgs::QuaternionStampedConstPtr& msg)
{
	if(!_got_inipose){
		tf::Quaternion q;
		quaternionMsgToTF(msg->quaternion, q);
		double r, p, y;
		tf::Matrix3x3(q).getRPY(r, p, y);
		_x <<
			r,
			p,
			y;
		_got_inipose = true;
	} 
}

void ImuEKFRPY::callbackIMU(const sensor_msgs::ImuConstPtr& msg)
{
	/*wait initial orientation*/
	if(!_got_inipose)	return;
	/*skip first callback*/
	if(!_got_first_imu){
		_imu_last = *msg;
		_got_first_imu = true;
		return;
	}
	/*get dt*/
	double dt;
	try{
		dt = (msg->header.stamp - _imu_last.header.stamp).toSec();
	}
	catch(std::runtime_error& ex) {
		ROS_ERROR("Exception: [%s]", ex.what());
		return;
	}
	/*prediction*/
	predictionIMU(*msg, dt);
	/*observation*/
	observationIMU(*msg);
	/*publication*/
	publication(msg->header.stamp);
	/*reset*/
	_imu_last = *msg;
}

void ImuEKFRPY::callbackBias(const sensor_msgs::ImuConstPtr& msg)
{
	if(!_got_bias){
		_bias = *msg;
		_got_bias = true;
	}
}

void ImuEKFRPY::predictionIMU(sensor_msgs::Imu imu, double dt)
{
	/*u*/
	Eigen::Vector3d u;
	// u <<
	// 	imu.angular_velocity.x*dt,
	// 	imu.angular_velocity.y*dt,
	// 	imu.angular_velocity.z*dt;
	u <<
		(imu.angular_velocity.x + _imu_last.angular_velocity.x)*dt/2.0,
		(imu.angular_velocity.y + _imu_last.angular_velocity.y)*dt/2.0,
		(imu.angular_velocity.z + _imu_last.angular_velocity.z)*dt/2.0;
	if(_got_bias){
		Eigen::Vector3d b;
		b <<
			_bias.angular_velocity.x*dt,
			_bias.angular_velocity.y*dt,
			_bias.angular_velocity.z*dt;
		u = u - b;
	}
	/*f*/
	Eigen::VectorXd f(_x.size());
	if(_use_quaternion_for_rotation){
		tf::Quaternion q_pose = tf::createQuaternionFromRPY(_x(0), _x(1), _x(2));
		tf::Quaternion q_rel_rot = tf::createQuaternionFromRPY(u(0), u(1), u(2));
		q_pose = q_pose*q_rel_rot;
		q_pose.normalize();
		tf::Matrix3x3(q_pose).getRPY(f(0), f(1), f(2));
	}
	else{
		Eigen::MatrixXd Rot(_x.size(), u.size());
		getRotMatrixRPY(_x(0), _x(1), Rot);
		f = _x + Rot*u;
		/*-pi to pi*/
		for(int i=0; i<f.size(); ++i)	anglePiToPi(f(i));
	}
	/*jF*/
	Eigen::MatrixXd jF(_x.size(), _x.size());
	jF(0, 0) = 1 + u(1)*cos(_x(0))*tan(_x(1)) - u(2)*sin(_x(0))*tan(_x(1));
	jF(0, 1) = u(1)*sin(_x(0))/cos(_x(1))/cos(_x(1)) + u(2)*cos(_x(0))/cos(_x(1))/cos(_x(1));
	jF(0, 2) = 0;
	jF(1, 0) = -u(1)*sin(_x(0)) - u(2)*cos(_x(0));
	jF(1, 1) = 1;
	jF(1, 2) = 0;
	jF(2, 0) = u(1)*cos(_x(0))/cos(_x(1)) - u(2)*sin(_x(0))/cos(_x(1));
	jF(2, 1) = u(1)*sin(_x(0))*sin(_x(1))/cos(_x(1))/cos(_x(1)) + u(2)*cos(_x(0))*sin(_x(1))/cos(_x(1))/cos(_x(1));
	jF(2, 2) = 1;
	/*Q*/
	Eigen::MatrixXd Q = _sigma_pre*Eigen::MatrixXd::Identity(_x.size(), _x.size());
	/*Update*/
	_x = f;
	_P = jF*_P*jF.transpose() + Q;
}

void ImuEKFRPY::observationIMU(sensor_msgs::Imu imu)
{
	/*
	std::cout << "- observationIMU -" << std::endl;
	std::cout
		<< "r[deg]: " << _x(0)/M_PI*180.0
		<< ", "
		<< "p[deg]: " << _x(1)/M_PI*180.0
		<< ", "
		<< "y[deg]: " << _x(2)/M_PI*180.0
	<< std::endl;
	*/
	/*z*/
	Eigen::Vector3d z(
		-imu.linear_acceleration.x,
		-imu.linear_acceleration.y,
		-imu.linear_acceleration.z
	);
	z.normalize();
	std::cout << "z: " << z(0) << ", " << z(1) << ", " << z(2) << std::endl;
	/*zp*/
	const double g = -1.0;
	Eigen::Vector3d zp(
		-g*sin(_x(1)),
		g*sin(_x(0))*cos(_x(1)),
		g*cos(_x(0))*cos(_x(1))
	);
	std::cout << "zp: " << zp(0) << ", " << zp(1) << ", " << zp(2) << std::endl;
	/*jH*/
	Eigen::MatrixXd jH(z.size(), _x.size());
	jH <<
		0,							-g*cos(_x(1)),				0,
		g*cos(_x(0))*cos(_x(1)),	-g*sin(_x(0))*sin(_x(1)),	0,
		-g*sin(_x(0))*cos(_x(1)),	-g*cos(_x(0))*sin(_x(1)),	0;
	/*R*/
	Eigen::MatrixXd R = _sigma_obs*Eigen::MatrixXd::Identity(z.size(), z.size());
	/*I*/
	Eigen::MatrixXd I = Eigen::MatrixXd::Identity(_x.size(), _x.size());
	/*y, s, K*/
	Eigen::Vector3d y = z - zp;
	Eigen::MatrixXd S = jH*_P*jH.transpose() + R;
	Eigen::MatrixXd K = _P*jH.transpose()*S.inverse();
	/*update*/
	_x = _x + K*y;
	_P = (I - K*jH)*_P;
	/*-pi to pi*/
	for(int i=0;i<_x.size();++i)	anglePiToPi(_x(i));
}

void ImuEKFRPY::publication(ros::Time stamp)
{
	/*RPY*/
	tf::Quaternion q_rpy = tf::createQuaternionFromRPY(_x(0), _x(1), _x(2));
	geometry_msgs::QuaternionStamped q_rpy_msg;
	q_rpy_msg.header.frame_id = _frame_id;
	q_rpy_msg.header.stamp = stamp;
	q_rpy_msg.quaternion.x = q_rpy.x();
	q_rpy_msg.quaternion.y = q_rpy.y();
	q_rpy_msg.quaternion.z = q_rpy.z();
	q_rpy_msg.quaternion.w = q_rpy.w();
	_pub_quat.publish(q_rpy_msg);
	/*print*/
	// std::cout << "r[deg]: " << _x(0) << " p[deg]: " << _x(1) << "y[deg]: " << _x(2) << std::endl;
}

void ImuEKFRPY::getRotMatrixRPY(double r, double p, Eigen::MatrixXd& Rot)
{
	Rot <<
		1,	sin(r)*tan(p),	cos(r)*tan(p),
		0,	cos(r),			-sin(r),
		0,	sin(r)/cos(p),	cos(r)/cos(p);
}

void ImuEKFRPY::anglePiToPi(double& angle)
{
	angle = atan2(sin(angle), cos(angle)); 
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "imu_ekf_rpy");
	
	ImuEKFRPY imu_ekf_rpy;

	ros::spin();
}
