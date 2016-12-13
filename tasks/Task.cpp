/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "Task.hpp"

//#define DEBUG_PRINTS 1

#ifndef D2R
#define D2R M_PI/180.00 /** Convert degree to radian **/
#endif
#ifndef R2D
#define R2D 180.00/M_PI /** Convert radian to degree **/
#endif

using namespace dso_slam;

Task::Task(std::string const& name)
    : TaskBase(name)
{
}

Task::Task(std::string const& name, RTT::ExecutionEngine* engine)
    : TaskBase(name, engine)
{
}

Task::~Task()
{
}

void Task::delta_pose_samplesTransformerCallback(const base::Time &ts, const ::base::samples::RigidBodyState &delta_pose_samples_sample)
{
    #ifdef DEBUG_PRINTS
    RTT::log(RTT::Warning)<<"[DSO_SLAM DELTA_POSE_SAMPLES] Received time-stamp: "<<delta_pose_samples_sample.time.toMicroseconds()<<RTT::endlog();
    #endif

    Eigen::Affine3d tf_body_sensor; /** Transformer transformation **/
    /** Get the transformation Tbody_sensor **/
    if (_sensor_frame.value().compare(_body_frame.value()) == 0)
    {
        tf_body_sensor.setIdentity();
    }
    else if (!_sensor2body.get(ts, tf_body_sensor, false))
    {
        RTT::log(RTT::Fatal)<<"[DSO_SLAM FATAL ERROR] No transformation provided."<<RTT::endlog();
       return;
    }

    /** Set to identity if it is not initialized **/
    if (!base::isnotnan(this->tf_odo_sensor_sensor_1.matrix()))
    {
        this->tf_odo_sensor_sensor_1.setIdentity();
    }

    /** Accumulate the relative sensor to sensor transformation Tsensor_sensor(k-1) **/
    Eigen::Affine3d tf_body_body = delta_pose_samples_sample.getTransform();
    this->tf_odo_sensor_sensor_1 = this->tf_odo_sensor_sensor_1 * (tf_body_sensor.inverse() * tf_body_body.inverse() * tf_body_sensor);
}

void Task::left_frameTransformerCallback(const base::Time &ts, const ::RTT::extras::ReadOnlyPointer< ::base::samples::frame::Frame > &left_frame_sample)
{
    #ifdef DEBUG_PRINTS
    std::cout << "[DSO_SLAM LEFT_FRAME] Frame arrived at: " <<left_frame_sample->time.toString()<< std::endl;
    #endif

    /** Convert image to gray scale **/
    this->left_frame.init(left_frame_sample->size.width, left_frame_sample->size.height, left_frame_sample->getDataDepth(), base::samples::frame::MODE_GRAYSCALE);
    this->frameHelperLeft.convert (*left_frame_sample, this->left_frame, 0, 0, _resize_algorithm.value(), true);

    this->process(this->left_frame, ts);
}

/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See Task.hpp for more detailed
// documentation about them.

bool Task::configureHook()
{
    if (! TaskBase::configureHook())
        return false;

    /** Frame index **/
    this->frame_idx = 0;

    /** Initialize the direct sparse odometry **/
    this->dso.reset(new ::dso::FullSystem());
    this->dso->linearizeOperation = false;

    /** Initialize output frame **/
    ::base::samples::frame::Frame *outframe = new ::base::samples::frame::Frame();
    this->frame_out.reset(outframe);
    outframe = NULL;

    /** Set Tsensor_sensor(k-1) from odometry to Nan **/
    this->tf_odo_sensor_sensor_1.matrix()= Eigen::Matrix<double, 4, 4>::Zero() * base::NaN<double>();

    return true;
}
bool Task::startHook()
{
    if (! TaskBase::startHook())
        return false;
    return true;
}
void Task::updateHook()
{
    TaskBase::updateHook();
}
void Task::errorHook()
{
    TaskBase::errorHook();
}
void Task::stopHook()
{
    TaskBase::stopHook();
}
void Task::cleanupHook()
{
    TaskBase::cleanupHook();

    /** Reset DSO **/
    this->dso.reset();
}

void Task::process(const base::samples::frame::Frame &frame_left,
                const base::Time &timestamp)
{
    /** Convert Images to opencv **/
    cv::Mat img_l = frameHelperLeft.convertToCvMat(frame_left);

    ::dso::MinimalImageB min_img((int)img_l.cols, (int)img_l.rows,(unsigned char*)img_l.data);
    ::dso::ImageAndExposure* undist_img = this->undistorter->undistort<unsigned char>(&min_img, 1,0, 1.0f);
    this->dso->addActiveFrame(undist_img, this->frame_idx);

    this->frame_idx++;
    delete undist_img;
}
