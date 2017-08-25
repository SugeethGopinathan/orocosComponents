/* Author: Sugeeth Gopinathan
 * Date:   25/07/2017
 *
 */

#ifndef RTTMANIPULABILITYESTIMATOR_HPP
#define RTTMANIPULABILITYESTIMATOR_HPP

// RTT header files. Might missing some or some be unused

#include <rtt/RTT.hpp>
#include <string>
#include <kdl/chain.hpp>
//#define ROBOT_DOF_SIZE 7
#include<rst-rt/geometry/Pose.hpp>

using namespace  KDL;

class RTTManipulabilityEstimator: public RTT::TaskContext {
public:
    RTTManipulabilityEstimator(std::string const & name);

    // RTT::TaskContext methods that are needed for any standard component and
    // should be implemented by user
    bool configureHook();
    bool startHook();
    void updateHook();
    void stopHook();
    void cleanupHook();

private:

    // Other variables

    rstrt::geometry::Pose shoulderVal, shoulderBase;

    Eigen::VectorXf paramVal, taskVal, taskArray;


    float userHeight, userArm1, userArm2, userDist;
    // Defining the input ports

    RTT::InputPort< rstrt::geometry::Pose> input_shoulder;
    RTT::InputPort< Eigen::VectorXf> input_taskarray;
    RTT::InputPort< Eigen::VectorXf> input_armparameters;

    // Defining the output ports

    RTT::OutputPort< double> output_manip;

	// Declare ports and their datatypes
    //KDL::Chain;
};

#endif // RTTMANIPULABILITYESTIMATOR_HPP
