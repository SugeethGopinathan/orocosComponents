#include <kinematic_chain.h>
#include <pid_values_tmp.h>

using namespace cogimon;
KinematicChain::KinematicChain(const std::string& chain_name,
		const std::vector<std::string> &joint_names,
		RTT::DataFlowInterface& ports, friRemote* friInst) :
		_kinematic_chain_name(chain_name), _ports(ports), _current_control_mode(
				std::string(ControlModes::JointPositionCtrl)), _joint_names(
				joint_names) {
	//move pointer for _fri_inst
	this->_fri_inst = friInst;
	char str[INET_ADDRSTRLEN];
	inet_ntop(AF_INET, &(_fri_inst->remote.krcAddr.sin_addr), str,
	INET_ADDRSTRLEN);


    RTT::log(RTT::Info) << "Opening FRI Version "
                        << FRI_MAJOR_VERSION << "." << FRI_SUB_VERSION << "." <<FRI_DATAGRAM_ID_CMD << "." <<FRI_DATAGRAM_ID_MSR
                  << " Interface for LWR ROS server" << RTT::endlog();




    RTT::log(RTT::Info) << "Checking if the robot is Stopped..." << RTT::endlog();
    if( this->_fri_inst->getState() == FRI_STATE_OFF )
    {
     RTT::log(RTT::Info) << "Please, start the KRL script now." << RTT::endlog();
    }




	//store krc_ip address in case it's needed
	this->_krc_ip = std::string(str);
	RTT::log(RTT::Info) << "Creating Kinematic Chain " << chain_name
			<< RTT::endlog();
	//changed from string string pair to string int as FRI doesn't use joint names but rather indexed positions (could change back to string and convert to int again?)
	for (unsigned int i = 0; i < _joint_names.size(); ++i)
		_map_joint_name_index.insert(
				std::pair<std::string, int>(_joint_names[i], i));

	RTT::log(RTT::Info) << "Joints: " << RTT::endlog();
	for (unsigned int i = 0; i < _joint_names.size(); ++i)
		RTT::log(RTT::Info) << "    " << _joint_names[i] << RTT::endlog();

	_initial_joints_configuration.reserve(joint_names.size());
	for (unsigned int i = 0; i < joint_names.size(); ++i)
		_initial_joints_configuration.push_back(0.0);
}

std::vector<RTT::base::PortInterface*> KinematicChain::getAssociatedPorts() {
	return _inner_ports;
}

bool KinematicChain::initKinematicChain() {
	RTT::log(RTT::Info) << "DEBUG1" << RTT::endlog();
	setJointNamesAndIndices();
	_number_of_dofs = _map_joint_name_index.size();
	RTT::log(RTT::Info) << "DEBUG2 :" << _number_of_dofs << RTT::endlog();
	//set last position memory for torque mode
	_last_pos = new float[_number_of_dofs];
	memset(_last_pos, 0, _number_of_dofs * sizeof(*_last_pos));
	RTT::log(RTT::Info) << "DEBUG3" << RTT::endlog();
	//set zero vector for torque mode 0 vectors
	zero_vector = new float[_number_of_dofs];
	RTT::log(RTT::Info) << "DEBUG4" << RTT::endlog();
//    memset(zero_vector, 0.00001, _number_of_dofs * sizeof(*zero_vector));
    memset(zero_vector, 0, _number_of_dofs * sizeof(*zero_vector));
	_joint_pos.resize(_number_of_dofs);
	_joint_pos.setZero();
	_joint_trq.resize(_number_of_dofs);
	_joint_trq.setZero();
	_joint_stiff.resize(_number_of_dofs);
	_joint_stiff.setZero();
	_joint_damp.resize(_number_of_dofs);
	_joint_damp.setZero();



    //Cartesian Components
    //TODO make sure the vector sizes below don't need to be expressed as variables
    _cartesian_pose.rotation.rotation.resize(4);
    _cartesian_pose.rotation.rotation.setZero();
    _cartesian_pose.translation.translation.resize(3);
    _cartesian_pose.translation.translation.setZero();
    _cartesian_damp.resize(6);
    _cartesian_damp.setZero();
    _cartesian_stiff.resize(6);
    _cartesian_stiff.setZero();


    trans.translation.resize(3);  // ToDo: See if this needs to be reseted to the current Pose ????
    trans.translation.setZero(3);
    rot.rotation.resize(4);
    rot.rotation.setZero(4);


    _transformation_matrix.resize(12);
    _transformation_matrix.setZero(12);


	RTT::log(RTT::Info) << "DEBUG5" << RTT::endlog();
	if (!setController(std::string(ControlModes::JointPositionCtrl)))
		return false;
	if (!setController(std::string(ControlModes::JointImpedanceCtrl)))
		return false;
	if (!setController(std::string(ControlModes::JointTorqueCtrl)))
		return false;
    if (!setController(std::string(ControlModes::CartesianImpedanceCtrl)))
        return false;

	RTT::log(RTT::Info) << "DEBUG6" << RTT::endlog();
	setFeedBack();
    setPoseFeedback();
	RTT::log(RTT::Info) << "DEBUG7" << RTT::endlog();
	//TODO change to init krc conrtoller (Possibly may not need anymore but needs adapting to new KRC.src program)
	if (!initKRC()) {
		return false;
	}
	setInitialPosition(false);
	RTT::log(RTT::Info) << "DEBUG8" << RTT::endlog();
	setInitialImpedance();
	RTT::log(RTT::Info) << "DEBUG9" << RTT::endlog();
	return true;
}

bool KinematicChain::resetKinematicChain() {
	setControlMode(ControlModes::JointPositionCtrl);
	setInitialPosition(false);
	setInitialImpedance();
	return true;
}

//scoped names actually become ints for fri
std::vector<int> KinematicChain::getJointScopedNames() {
	std::vector<int> joint_names;
	for (unsigned int i = 0; i < _joint_names.size(); ++i)
		joint_names.push_back(_map_joint_name_index.at(_joint_names[i]));
	return joint_names;
}

std::vector<std::string> KinematicChain::getJointNames() {
	std::vector<std::string> joint_names;
	for (unsigned int i = 0; i < _joint_names.size(); ++i)
		joint_names.push_back(_joint_names[i]);
	return joint_names;
}

std::vector<std::string> KinematicChain::getControllersAvailable() {
	return _controllers_name;
}

std::string KinematicChain::getKinematicChainName() {
	return _kinematic_chain_name;
}

unsigned int KinematicChain::getNumberOfDOFs() {
	return _number_of_dofs;
}

std::string KinematicChain::getCurrentControlMode() {
	return _current_control_mode;
}

void KinematicChain::setFeedBack() {
	full_feedback.reset(new full_fbk);
	full_feedback->orocos_port.setName(
			_kinematic_chain_name + "_JointFeedback");
	full_feedback->orocos_port.doc(
			"Output for Joint-fb from FRI to Orocos world. Contains joint-position, -velocity and -torque.");
	_ports.addPort(full_feedback->orocos_port);
	// TODO needs to be solved better
	_inner_ports.push_back(
			_ports.getPort(full_feedback->orocos_port.getName()));

	full_feedback->joint_feedback = JointState(_number_of_dofs);
	full_feedback->orocos_port.setDataSample(full_feedback->joint_feedback);
}

void KinematicChain::setPoseFeedback() {
    cartesian_feedback.reset(new pose_fbk);

    cartesian_feedback->orocos_port.setName(
                _kinematic_chain_name + "_PoseFeedback");
    cartesian_feedback->orocos_port.doc(
                "Output for cartesian poses from FRI to Orocos world, contains Cartesian poses");
    _ports.addPort(cartesian_feedback->orocos_port);
    _inner_ports.push_back(
                _ports.getPort(cartesian_feedback->orocos_port.getName()));



    //cartesian_feedback->pose_feedback = Eigen::VectorXf(12);  // change this to pose, if the cartesian pose is being sent out

    cartesian_feedback->pose_feedback = rstrt::geometry::Pose ( trans,rot );
    cartesian_feedback->orocos_port.setDataSample(cartesian_feedback->pose_feedback);


}

bool KinematicChain::setController(const std::string& controller_type) {
	if (controller_type == ControlModes::JointPositionCtrl) {
		position_controller.reset(new position_ctrl);
		position_controller->orocos_port.setName(
				_kinematic_chain_name + "_" + ControlModes::JointPositionCtrl);
		position_controller->orocos_port.doc(
				"Input for JointPosition-cmds from Orocos to FRI world.");
		_ports.addPort(position_controller->orocos_port);
		// TODO needs to be solved better
		_inner_ports.push_back(
				_ports.getPort(position_controller->orocos_port.getName()));

		position_controller->joint_cmd = JointAngles(_number_of_dofs);
		position_controller->joint_cmd.angles.setZero();
	} else if (controller_type == ControlModes::JointImpedanceCtrl) {
		impedance_controller.reset(new impedance_ctrl);
		impedance_controller->orocos_port.setName(
				_kinematic_chain_name + "_" + ControlModes::JointImpedanceCtrl);
		impedance_controller->orocos_port.doc(
				"Input for JointImpedance-cmds from Orocos to FRI world.");
		_ports.addPort(impedance_controller->orocos_port);
		// TODO needs to be solved better
		_inner_ports.push_back(
				_ports.getPort(impedance_controller->orocos_port.getName()));

		impedance_controller->joint_cmd = JointImpedance(_number_of_dofs);
		impedance_controller->joint_cmd.stiffness.setZero();
		impedance_controller->joint_cmd.damping.setZero();
	} else if (controller_type == ControlModes::JointTorqueCtrl) {
		torque_controller.reset(new torque_ctrl);
		torque_controller->orocos_port.setName(
				_kinematic_chain_name + "_" + ControlModes::JointTorqueCtrl);
		torque_controller->orocos_port.doc(
				"Input for JointTorque-cmds from Orocos to FRI world.");
		_ports.addPort(torque_controller->orocos_port);
		// TODO needs to be solved better
		_inner_ports.push_back(
				_ports.getPort(torque_controller->orocos_port.getName()));

		torque_controller->joint_cmd = JointTorques(_number_of_dofs);
		torque_controller->joint_cmd.torques.setZero();
    }

    // Cartesian Impedance Set Control
    else if (controller_type == ControlModes::CartesianImpedanceCtrl) {
        cartesian_impedance_controller.reset(new cart_impedance_ctrl);
        cartesian_impedance_controller->orocos_port.setName(
                    _kinematic_chain_name+"_"+ControlModes::CartesianImpedanceCtrl);
        cartesian_impedance_controller->orocos_port.doc(
                    "Input for CartesianImpedance-cmds from Orocos to FRI world.");
        _ports.addPort(cartesian_impedance_controller->orocos_port);
        _inner_ports.push_back(
                    _ports.getPort(cartesian_impedance_controller->orocos_port.getName()));

        cartesian_impedance_controller->joint_cmd = Pose(trans,rot); // Here it could be EigenVector in case we are pushing a 4*3 matrix else a Pose if we are giving in a pose
        cartesian_impedance_controller->joint_cmd.translation.translation.setZero();
        cartesian_impedance_controller->joint_cmd.rotation.rotation.setZero();



    }



    else {
		RTT::log(RTT::Error) << "Control Mode: " << controller_type
				<< " is not available!" << RTT::endlog();
		return false;
	}
	_controllers_name.push_back(controller_type);
	return true;
}

bool KinematicChain::setJointNamesAndIndices() {
	std::map<std::string, int>::iterator it1;
	for (it1 = _map_joint_name_index.begin();
			it1 != _map_joint_name_index.end(); it1++) {
		_map_joint_name_index[it1->first] = it1->second;

		RTT::log(RTT::Info) << "Joint " << it1->first << " in chain "
				<< _kinematic_chain_name << " has scoped name "
				<< _map_joint_name_index[it1->first] << RTT::endlog();
	}
	return true;
}

void KinematicChain::setInitialPosition(const bool use_actual_model_pose) {
	position_controller->orocos_port.clear();
	RTT::log(RTT::Info) << _joint_names.size()<<" joint size" << RTT::endlog();
	//get initial positoin from fri, currently using loop possibly convert to Eigen::map or something?
	if (use_actual_model_pose) {
		for (unsigned int i = 0; i < _joint_names.size(); ++i)
			position_controller->joint_cmd.angles[i] =
					_fri_inst->getMsrMsrJntPosition()[i];
	} else {
		for (unsigned int i = 0; i < _joint_names.size(); ++i){
			RTT::log(RTT::Info) << position_controller->joint_cmd.angles.size()<<" loop, "<<
					_initial_joints_configuration.size()<< RTT::endlog();
			position_controller->joint_cmd.angles[i] =
					_initial_joints_configuration.at(i);}
	}
	position_controller->joint_cmd_fs = RTT::FlowStatus::NewData;
}

void KinematicChain::setInitialImpedance() {
	hardcoded_impedance impedance_init;
	for (unsigned int i = 0; i < _joint_names.size(); ++i) {
		impedance_controller->joint_cmd.stiffness[i] =
				impedance_init.impedance[_joint_names[i]].first;
		impedance_controller->joint_cmd.damping[i] =
				impedance_init.impedance[_joint_names[i]].second;
	}
	impedance_controller->joint_cmd_fs = RTT::FlowStatus::NewData;
}

bool KinematicChain::setControlMode(const std::string &controlMode) {
	if (controlMode != ControlModes::JointPositionCtrl
			&& controlMode != ControlModes::JointTorqueCtrl
            && controlMode != ControlModes::JointImpedanceCtrl
            && controlMode != ControlModes::CartesianImpedanceCtrl
            ) {
		RTT::log(RTT::Warning) << "Control Mode " << controlMode
				<< " does not exist!" << RTT::endlog();
		return false;
	}

	if (!(std::find(_controllers_name.begin(), _controllers_name.end(),
			controlMode) != _controllers_name.end())) {
		RTT::log(RTT::Warning) << "Control Mode " << controlMode
				<< " is not available!" << RTT::endlog();
		return false;
	}
	//TODO add all control modes in properly with different options between them
	//set control modes and the correct cmdFlags as well (must be done in monitor mode not command mode! and can't be changed in command mode)	
	if (controlMode == ControlModes::JointPositionCtrl) {
		_fri_inst->getCmdBuf().cmd.cmdFlags = FRI_CMD_JNTPOS;
		_fri_inst->setToKRLInt(14, 10);
	} else if (controlMode == ControlModes::JointTorqueCtrl) {
		_fri_inst->getCmdBuf().cmd.cmdFlags = 0;
		_fri_inst->getCmdBuf().cmd.cmdFlags |= FRI_CMD_JNTPOS;
		_fri_inst->getCmdBuf().cmd.cmdFlags |= FRI_CMD_JNTSTIFF;
		_fri_inst->getCmdBuf().cmd.cmdFlags |= FRI_CMD_JNTDAMP;
		_fri_inst->getCmdBuf().cmd.cmdFlags |= FRI_CMD_JNTTRQ;
		_fri_inst->setToKRLInt(14, 30);
	}

    else if (controlMode == ControlModes::CartesianImpedanceCtrl){

        _fri_inst->getCmdBuf().cmd.cmdFlags = 0;
        _fri_inst->getCmdBuf().cmd.cmdFlags |= FRI_CMD_CARTPOS;
        _fri_inst->getCmdBuf().cmd.cmdFlags |= FRI_CMD_CARTSTIFF;
        _fri_inst->getCmdBuf().cmd.cmdFlags |= FRI_CMD_CARTDAMP;
        // DO I NEED Torque too?
        _fri_inst->getCmdBuf().cmd.cmdFlags |= FRI_CMD_TCPFT;
        _fri_inst->getCmdBuf().cmd.cmdFlags |= FRI_CMD_JNTPOS;                          //FRI Nullspace needed too??
        _fri_inst->setToKRLInt(14,20);  // where to get this value
    }
//	else if (controlMode == ControlModes::JointTorqueCtrl
//			|| controlMode == ControlModes::JointImpedanceCtrl)
//		_gazebo_position_joint_controller->Reset();
	_current_control_mode = controlMode;
	return true;
}

void KinematicChain::sense() {
	//recieve data from fri
	_fri_inst->doReceiveData();
    //set mode command to zero so fri does not continue if it is being executed again (possibly move to component stop method)
    _fri_inst->setToKRLInt(15, 0);
	//if not in monitor mode straight return as nothing can be sensed
    if (_fri_inst->getFrmKRLInt(15) < 10) {
		return;
	}
	//if in monitor mode command fri to switch to command mode with the correct control mode
	if (_fri_inst->getFrmKRLInt(15) == 10) {
        // call startFRI with this command below!
		_fri_inst->setToKRLInt(15, 10);
		//return;
	}
	if (full_feedback) {
		time_now = RTT::os::TimeService::Instance()->getNSecs();
		//get the current pos
		for (unsigned int i = 0; i < _number_of_dofs; ++i)
			full_feedback->joint_feedback.angles(i) =
					_fri_inst->getMsrMsrJntPosition()[i];

		//TODO calculate velocity feedback! Easy way first, maybe Kalman afterwards?

		for (unsigned int i = 0; i < _number_of_dofs; ++i) {
			full_feedback->joint_feedback.velocities(i) =
					(_fri_inst->getMsrMsrJntPosition()[i] - _last_pos[i])
							/ ((time_now - last_time)*1E-9);
			_last_pos[i] = _fri_inst->getMsrMsrJntPosition()[i];
		}
		//get the current torques
		for (unsigned int i = 0; i < _number_of_dofs; ++i) {
			full_feedback->joint_feedback.torques(i) =
					_fri_inst->getMsrJntTrq()[i];
		}

		if (full_feedback->orocos_port.connected())
			full_feedback->orocos_port.write(full_feedback->joint_feedback);
		last_time = time_now;
	}

    if(cartesian_feedback){

        RTT::log(RTT::Info) << "CARTESIAN FEEDBACK RECEIVING"<<_kinematic_chain_name<< RTT::endlog();
        time_now = RTT::os::TimeService::Instance()->getNSecs();

        // Get the Cartesian Pose from the FRI and translate it to pose
        Eigen::VectorXf pose_update(12);
        pose_update.setZero();

        for(int i= 0 ;i <12; i++)
        {

            pose_update(i)= _fri_inst->getMsrCartPosition()[i];
            //_fri_inst->get
        }


        // Carteian pose calculation from the Rotation matrix read from the FRI

        cartesian_feedback->pose_feedback.translation.translation(0)= pose_update(3);
        cartesian_feedback->pose_feedback.translation.translation(1)= pose_update(7);
        cartesian_feedback->pose_feedback.translation.translation(2)= pose_update(11);


        cartesian_feedback->pose_feedback.rotation.rotation(0)= sqrt(1.0+pose_update(0)+pose_update(5)+pose_update(10));
        double w4 = 4 * sqrt(1.0+pose_update(0)+pose_update(5)+pose_update(10));
        cartesian_feedback->pose_feedback.rotation.rotation(1)= (pose_update(9)-pose_update(6))/w4;
        cartesian_feedback->pose_feedback.rotation.rotation(2)= (pose_update(2)-pose_update(8))/w4;
        cartesian_feedback->pose_feedback.rotation.rotation(3)= (pose_update(4)-pose_update(1))/w4;

        if(cartesian_feedback->orocos_port.connected())
            cartesian_feedback->orocos_port.write(cartesian_feedback->pose_feedback);

        last_time = time_now;
    }

}

void KinematicChain::getCommand() {
	if (_current_control_mode == ControlModes::JointTorqueCtrl)
		torque_controller->joint_cmd_fs =
				torque_controller->orocos_port.readNewest(
						torque_controller->joint_cmd);
	else if (_current_control_mode == ControlModes::JointPositionCtrl)
		position_controller->joint_cmd_fs =
				position_controller->orocos_port.readNewest(
						position_controller->joint_cmd);
	else if (_current_control_mode == ControlModes::JointImpedanceCtrl) {
		position_controller->joint_cmd_fs =
				position_controller->orocos_port.readNewest(
						position_controller->joint_cmd);
		impedance_controller->joint_cmd_fs =
				impedance_controller->orocos_port.readNewest(
						impedance_controller->joint_cmd);
		torque_controller->joint_cmd_fs =
				torque_controller->orocos_port.readNewest(
						torque_controller->joint_cmd);
	}
    else if(_current_control_mode == ControlModes::CartesianImpedanceCtrl){

        // Using the port of type Pose to get the cartesian Pose and the Port of type JoinImpedance to get the stiffness and damping values
        cartesian_impedance_controller->joint_cmd_fs =
                cartesian_impedance_controller->orocos_port.readNewest(
                    cartesian_impedance_controller->joint_cmd);
        impedance_controller->joint_cmd_fs =
                impedance_controller->orocos_port.readNewest(
                    impedance_controller->joint_cmd);
        // TODO:

        // Add the provision for Nullspace port later


    }
}

void KinematicChain::move() {
    RTT::log(RTT::Info) <<"\n"<<_fri_inst->getCmdBuf().cmd.cmdFlags<<"\n"<<_fri_inst->getCurrentCommandFlags()<<RTT::endlog();
	/*if(_fri_inst->getQuality()!= FRI_QUALITY::FRI_QUALITY_PERFECT){

	 return;
	 }*/
//only run when KRC is in command mode (don't need to check for perfect communication as the program will only go into command mode when communication is perfect)
	if (_fri_inst->getFrmKRLInt(15) == 20) {
		if (_current_control_mode == ControlModes::JointPositionCtrl) {
			//if(_fri_inst->getCurrentControlScheme()
			//		!= FRI_CTRL::FRI_CTRL_POSITION){
			//	_fri_inst->setToKRLInt(1, 10);
			//}else{
			//Currently done on index only, maybe convert to names as well?
//		_fri_inst->doPositionControl(
//				position_controller->joint_cmd.angles.data(), false);
			std::vector<int> joint_scoped_names = getJointScopedNames();
			for (unsigned int i = 0; i < joint_scoped_names.size(); ++i) {
				_joint_pos(joint_scoped_names[i]) =
						position_controller->joint_cmd.angles(i);
			}
			if (position_controller->joint_cmd_fs == RTT::NewData) {
				_fri_inst->doPositionControl(_joint_pos.data(), false);
			}
		} else if (_current_control_mode == ControlModes::JointTorqueCtrl) {
			//if(_fri_inst->getCurrentControlScheme()
			//		!= FRI_CTRL::FRI_CTRL_JNT_IMP){
			//	_fri_inst->setToKRLInt(1, 30);
			//}else{            
			std::vector<int> joint_scoped_names = getJointScopedNames();
			for (unsigned int i = 0; i < joint_scoped_names.size(); ++i) {
                _joint_trq(joint_scoped_names[i]) =
                        torque_controller->joint_cmd.torques(i);
			}

            //if (torque_controller->joint_cmd_fs == RTT::NewData) {
                RTT::log(RTT::Info) << "tau:\n" << _joint_trq << RTT::endlog();
                _fri_inst->doJntImpedanceControl(_fri_inst->getMsrMsrJntPosition(),
                    zero_vector, zero_vector, _joint_trq.data(), false);
            //}
//}
		} else if (_current_control_mode == ControlModes::JointImpedanceCtrl) {
//if(_fri_inst->getCurrentControlScheme()
			//			!= FRI_CTRL::FRI_CTRL_JNT_IMP){
			//	_fri_inst->setToKRLInt(1, 30);
			//	}else{
			std::vector<int> joint_scoped_names = getJointScopedNames();
			for (unsigned int i = 0; i < joint_scoped_names.size(); ++i) {
				_joint_pos(joint_scoped_names[i]) =
						position_controller->joint_cmd.angles(i);
				_joint_stiff(joint_scoped_names[i]) =
						impedance_controller->joint_cmd.stiffness(i);
				_joint_damp(joint_scoped_names[i]) =
						impedance_controller->joint_cmd.damping(i);
				_joint_trq(joint_scoped_names[i]) =
						torque_controller->joint_cmd.torques(i);
			}
			_fri_inst->doJntImpedanceControl(_joint_pos.data(),
					_joint_stiff.data(), _joint_damp.data(), _joint_trq.data(),
					false);
//}
		}

        else if (_current_control_mode == ControlModes::CartesianImpedanceCtrl) {



               /*      Transformation of Input pose into Transformation Matrix needed by FRI
                    base_position(0) = cartesian_impedance_controller->joint_cmd.translation.translation(0);
                    base_position(1) = cartesian_impedance_controller->joint_cmd.translation.translation(1);
                    base_position(2) = cartesian_impedance_controller->joint_cmd.translation.translation(2);

                    Eigen::Quaternionf qt;
                    qt.w()= cartesian_impedance_controller->joint_cmd.rotation.rotation(0);
                    qt.x()= cartesian_impedance_controller->joint_cmd.rotation.rotation(1);
                    qt.y()= cartesian_impedance_controller->joint_cmd.rotation.rotation(2);
                    qt.z()= cartesian_impedance_controller->joint_cmd.rotation.rotation(3);

                    qt.normalize();

                    float d0 = 1 - 2*qt.y()*qt.y() - 2 *qt.z()*qt.z();
                    float d1 = 2 * qt.x()*qt.y() - 2*qt.z()*qt.w();
                    float d2 = 2 * qt.x() * qt.z() + 2 * qt.y() * qt.w();
                    float d3 = 2 * qt.x() * qt.y() + 2 * qt.z() * qt.w();
                    float d4 = 1 - 2*qt.x()*qt.x() - 2 * qt.z()*qt.z();
                    float d5 = 2 * qt.y() * qt.z() - 2 * qt.x() * qt.w();
                    float d6 = 2 * qt.x() * qt.z() - 2 * qt.y() * qt.w();
                    float d7 = 2 * qt.y() * qt.z() + 2 * qt.x() * qt.w();
                    float d8 = 1 - 2 * qt.x()*qt.x() - 2 * qt.y()*qt.y();

                    _transformation_matrix[0] = d0;
                    _transformation_matrix[1] = d1;
                    _transformation_matrix[2] = d2;
                    _transformation_matrix[3] = base_position(0);
                    _transformation_matrix[4] = d3;
                    _transformation_matrix[5] = d4;
                    _transformation_matrix[6] = d5;
                    _transformation_matrix[7] =  base_position(1);
                    _transformation_matrix[8] = d6;
                    _transformation_matrix[9] = d7;
                    _transformation_matrix[10] = d8;
                    _transformation_matrix[11] = base_position(2);*/

                    for(int i=0;i<12;i++)
                    {
                        _transformation_matrix[i] =  _fri_inst->getMsrCartPosition()[i];


                    }



                    // Uncomment for accepting stiffness and damping values
                    /*for (unsigned int i = 0; i < 6; ++i) {


                        // Reading the commanded stiffness and the damping values 6 values represent 3 translational and 3 rotational values
                        _cartesian_stiff[i] =
                                impedance_controller->joint_cmd.stiffness(i);
                        _cartesian_damp[i] =
                                impedance_controller->joint_cmd.damping(i);

                    }*/

                    // Need to check if the NULL is a valid input in this case.


                    //TEST

                    for (unsigned int i = 0; i < 6; ++i) {


                        // Reading the commanded stiffness and the damping values 6 values represent 3 translational and 3 rotational values
                        _cartesian_stiff[i] =
                               1000.0;
                        _cartesian_damp[i] =0.7;

                    }



                    _fri_inst->doCartesianImpedanceControl(_transformation_matrix.data(),_cartesian_stiff.data(),_cartesian_damp.data(),NULL,NULL,false); // Nullspace and AddnewTCPFT made NULL, can be changed in future
                    //_fri_inst->doCartesianImpedanceControl(NULL,NULL,NULL,NULL,NULL,false);

                }
	} else {
		//if not in command mode run keep the jntPosition updated, required by FRI
		for (int i = 0; i < LBR_MNJ; i++) {
			_fri_inst->getCmdBuf().cmd.jntPos[i] =
					_fri_inst->getMsrBuf().data.cmdJntPos[i]
							+ _fri_inst->getMsrBuf().data.cmdJntPosFriOffset[i];
		}
	}

	//Send data
	_fri_inst->doSendData();
}

std::string KinematicChain::printKinematicChainInformation() {
	std::stringstream joint_names_stream;
	for (unsigned int i = 0; i < _joint_names.size(); ++i)
		joint_names_stream << _joint_names[i] << " ";

	std::vector<std::string> controller_names = getControllersAvailable();
	std::stringstream controller_names_stream;
	for (unsigned int i = 0; i < controller_names.size(); ++i)
		controller_names_stream << controller_names[i] << " ";

	std::stringstream info;
	info << "Kinematic Chain: " << _kinematic_chain_name << std::endl;
	info << "    Number of DOFs: " << _number_of_dofs << std::endl;
	info << "    Joints:  [" << joint_names_stream.str() << "]" << std::endl;
	info << "    Control Modes:  [ " << controller_names_stream.str() << "]"
			<< std::endl;
	info << "    Current Control Mode: " << _current_control_mode << std::endl;

	return info.str();
}

bool KinematicChain::initKRC() {
	last_quality = FRI_QUALITY_BAD;
	/*int i = 0;
	 RTT::log(RTT::Info) << "Trying to initialize"
	 << RTT::endlog();
	 last_quality = FRI_QUALITY_BAD;
	 //TODO do handshake as separate procedure?
	 while (_fri_inst->getQuality() < FRI_QUALITY::FRI_QUALITY_PERFECT || i<200) {
	 RTT::log(RTT::Info) << "Trying to initialize :"<<_fri_inst->getQuality()
	 << RTT::endlog();
	 _fri_inst->doReceiveData();
	 _fri_inst->setToKRLInt(0, 1);

	 last_quality = _fri_inst->getQuality();
	 if (last_quality >= FRI_QUALITY_OK ) {
	 i++;
	 _fri_inst->setToKRLInt(1, 10);
	 _fri_inst->doTest();
	 //_fri_inst->doDataExchange();
	 }
	 _fri_inst->setToKRLReal(1, _fri_inst->getFrmKRLReal(1));

	 _fri_inst->doSendData();
	 }*/


    // TODO dlw
    //this->_fri_inst->setToKRLInt(1, 1);
    //this->_fri_inst->doDataExchange();


    RTT::log(RTT::Info) << "FRI Status:\n" << this->_fri_inst->getMsrBuf().intf << RTT::endlog();
    RTT::log(RTT::Info) << "Sampling Rate: " << this->_fri_inst->getSampleTime() << RTT::endlog();


	return true;
}
