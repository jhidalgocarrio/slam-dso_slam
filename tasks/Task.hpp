/* Generated from orogen/lib/orogen/templates/tasks/Task.hpp */

#ifndef DSO_SLAM_TASK_TASK_HPP
#define DSO_SLAM_TASK_TASK_HPP

#include "dso_slam/TaskBase.hpp"

/** DSO library **/
#include <dso/util/settings.h>
#include <dso/FullSystem/FullSystem.h>
#include <dso/util/Undistort.h>

/** FrameHelper libraries **/
#include <frame_helper/FrameHelper.h> /** Rock lib for manipulate frames **/
#include <frame_helper/FrameHelperTypes.h> /** Types for FrameHelper **/

/** Boost **/
#include <boost/shared_ptr.hpp> /** shared pointers **/
#include <boost/math/special_functions/round.hpp> // to round a number in standard C++ < 11

namespace dso_slam{

    /*! \class Task
     * \brief The task context provides and requires services. It uses an ExecutionEngine to perform its functions.
     * Essential interfaces are operations, data flow ports and properties. These interfaces have been defined using the oroGen specification.
     * In order to modify the interfaces you should (re)use oroGen and rely on the associated workflow.
     * Declare a new task context (i.e., a component)

The corresponding C++ class can be edited in tasks/Task.hpp and
tasks/Task.cpp, and will be put in the dso_slam namespace.
     * \details
     * The name of a TaskContext is primarily defined via:
     \verbatim
     deployment 'deployment_name'
         task('custom_task_name','dso_slam::Task')
     end
     \endverbatim
     *  It can be dynamically adapted when the deployment is called with a prefix argument.
     */
    class Task : public TaskBase
    {
	friend class TaskBase;

    protected:
        /**************************/
        /*** Property Variables ***/
        /**************************/

        //Intrinsic and extrinsic parameters for the pinhole camera model
        frame_helper::StereoCalibration cameracalib;

        /******************************************/
        /*** General Internal Storage Variables ***/
        /******************************************/
        int frame_idx; // incremental stereo pair index

        frame_helper::FrameHelper frameHelperLeft; /** Frame helper **/

        base::samples::frame::Frame left_frame;

        boost::shared_ptr< ::dso::FullSystem > dso; /** Direct Sparse Odometry **/

        boost::shared_ptr< ::dso::Undistort > undistorter;

        Eigen::Affine3d tf_odo_sensor_sensor_1; // Relative camera transformations from delta_poses Tsensor(k)_sensor(k-1)

        /***************************/
        /** Output Port Variables **/
        /***************************/

        /** DSO features frame image **/
        RTT::extras::ReadOnlyPointer<base::samples::frame::Frame> frame_out;

    protected:

        virtual void delta_pose_samplesTransformerCallback(const base::Time &ts, const ::base::samples::RigidBodyState &delta_pose_samples_sample);

        virtual void left_frameTransformerCallback(const base::Time &ts, const ::RTT::extras::ReadOnlyPointer< ::base::samples::frame::Frame > &left_frame_sample);

    public:
        /** TaskContext constructor for Task
         * \param name Name of the task. This name needs to be unique to make it identifiable via nameservices.
         * \param initial_state The initial TaskState of the TaskContext. Default is Stopped state.
         */
        Task(std::string const& name = "dso_slam::Task");

        /** TaskContext constructor for Task
         * \param name Name of the task. This name needs to be unique to make it identifiable for nameservices.
         * \param engine The RTT Execution engine to be used for this task, which serialises the execution of all commands, programs, state machines and incoming events for a task.
         * 
         */
        Task(std::string const& name, RTT::ExecutionEngine* engine);

        /** Default deconstructor of Task
         */
	~Task();

        /** This hook is called by Orocos when the state machine transitions
         * from PreOperational to Stopped. If it returns false, then the
         * component will stay in PreOperational. Otherwise, it goes into
         * Stopped.
         *
         * It is meaningful only if the #needs_configuration has been specified
         * in the task context definition with (for example):
         \verbatim
         task_context "TaskName" do
           needs_configuration
           ...
         end
         \endverbatim
         */
        bool configureHook();

        /** This hook is called by Orocos when the state machine transitions
         * from Stopped to Running. If it returns false, then the component will
         * stay in Stopped. Otherwise, it goes into Running and updateHook()
         * will be called.
         */
        bool startHook();

        /** This hook is called by Orocos when the component is in the Running
         * state, at each activity step. Here, the activity gives the "ticks"
         * when the hook should be called.
         *
         * The error(), exception() and fatal() calls, when called in this hook,
         * allow to get into the associated RunTimeError, Exception and
         * FatalError states.
         *
         * In the first case, updateHook() is still called, and recover() allows
         * you to go back into the Running state.  In the second case, the
         * errorHook() will be called instead of updateHook(). In Exception, the
         * component is stopped and recover() needs to be called before starting
         * it again. Finally, FatalError cannot be recovered.
         */
        void updateHook();

        /** This hook is called by Orocos when the component is in the
         * RunTimeError state, at each activity step. See the discussion in
         * updateHook() about triggering options.
         *
         * Call recover() to go back in the Runtime state.
         */
        void errorHook();

        /** This hook is called by Orocos when the state machine transitions
         * from Running to Stopped after stop() has been called.
         */
        void stopHook();

        /** This hook is called by Orocos when the state machine transitions
         * from Stopped to PreOperational, requiring the call to configureHook()
         * before calling start() again.
         */
        void cleanupHook();

        /** @brief Process the images
         * */
        void process(const base::samples::frame::Frame &frame_left,
                const base::Time &timestamp);
    };
}

#endif

