/* Author: Sugeeth Gopinathan
 * Date:   25/07/2017
 *
 *The components reads the shoulder pose and the arm lengths of an user
 * the manipulability for a given task is calculated and pubished by this component
 */

#include "rtt-manipulability-estimator.hpp"
// needed for the macro at the end of this file:
#include <rtt/Component.hpp>

RTTManipulabilityEstimator::RTTManipulabilityEstimator(std::string const & name) : RTT::TaskContext(name) {
    // constructor:

    
}

bool RTTManipulabilityEstimator::configureHook() {
    // intializations and object creations go here. Each component should run this before being able to run
    return true;
}

bool RTTManipulabilityEstimator::startHook() {

    std::cout<< "Starting Manipulability Estimator"<<std::endl;

    if(!input_armparameters.connected() && !input_shoulder.connected() && !input_taskarray.connected())
    {
        Orocos::log(Orocos::Warning) <<"Input Ports Not connected" << Orocos::endlog();
        return false;

    }

    
    // Here we can read the arm parameters for setting up the kinematic chain
    
    if(input_armparameters.read(paramVal) == RTT::NewData){

        userHeight = paramVal(1);
        userArm1 = paramVal(2);
        userArm2 = paramVal(3);
        userDist = paramVal(4);

    }
    
    if(input_shoulder.read(shoulderVal) == RTT::NewData){

        shoulderBase = shoulderVal;


    }

    if(input_taskarray.read(taskVal) == RTT::NewData){

        taskArray = taskVal;

    }
    
    
    // Need to create a kinematic chain here
    
    KDL::Chain chain;
    chain.addSegment(Segment(Joint(Joint::RotZ),Frame(Vector(0.0,0.0,1.020))));
    chain.addSegment(Segment(Joint(Joint::RotX),Frame(Vector(0.0,0.0,0.480))));
    chain.addSegment(Segment(Joint(Joint::RotX),Frame(Vector(0.0,0.0,0.645))));
    chain.addSegment(Segment(Joint(Joint::RotZ)));
    chain.addSegment(Segment(Joint(Joint::RotX),Frame(Vector(0.0,0.0,0.120))));
    chain.addSegment(Segment(Joint(Joint::RotZ)));

    return true;


}

void RTTManipulabilityEstimator::updateHook() {


    // this is the actual body of a component. it is called on each cycle
}

void RTTManipulabilityEstimator::stopHook() {
    // stops the component (update hook wont be  called anymore)
}

void RTTManipulabilityEstimator::cleanupHook() {
    // cleaning the component data
}

// This macro, as you can see, creates the component. Every component should have this!
ORO_CREATE_COMPONENT_LIBRARY()ORO_LIST_COMPONENT_TYPE(RTTManipulabilityEstimator)
