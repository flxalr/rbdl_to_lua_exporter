#include <assert.h>
#include <iostream>
#include <fstream>
#include <stack>
#include <algorithm>

#include "Model_builder.h"

using namespace std;
using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;

Model_builder::Model_builder(string model_file)
{
    filename = model_file;
    active_set = false;
    fixed_set = false;
    gravity_set = false;
    set_half_bodies = false;
    set_planar_base = false;
}

Model_builder::~Model_builder()
{
}

void Model_builder::setActiveJoints(std::vector< std::string > user_joints)
{
  active_joints = user_joints;
  active_set = true;
}

void Model_builder::setFixedAngles(std::vector< std::string > joints, vector< double > angles)
{  
  if(joints.size() != angles.size())
  {
    cerr << "The sizes of joint list and angle list have to be the same!" << endl;
    abort();
  }
  
  // check that the joints in the given list are NOT in the active joints list , in case this is set
  if(active_set)
  {
    vector<std::string>::iterator it;
    bool found = false;
    for(int i = 0; i < joints.size(); i++)
    {
      it = std::find(active_joints.begin(), active_joints.end(),joints[i]);
      if(it != active_joints.end())
      {
	cerr << "Joint " << joints[i] << " appear to be in the active joints list." << endl;
	found = true;
      }
    }
    if(found)
    {
      abort();
    }
  }
  
  // save the lists
  joints_fixed_angle = joints;
  fixed_angles = angles;
  
  fixed_set = true;
}

void Model_builder::setGravity(Vector3d g)
{
  gravity_custom = g;
  gravity_set = true;
}

void Model_builder::setHalfBodies(std::vector< std::string > bodies)
{
  set_half_bodies = true;
  half_bodies = bodies;
}

void Model_builder::setPlanarBase()
{
  set_planar_base = true;
}

void Model_builder::build_model(Model* built_model, bool verbose)
{
  //check if the list of active joints has been specified
  if(active_joints.size() == 0)
  {
    cerr << "WARNING: you did NOT set the active joint list, the specified model will be used as it is." << endl;
  }
  
  //check if the model is initialized
  if(built_model->mBodies.size() > 1)
  {
    cerr << "WARNING: The model is not empty, it will be ovverwritten." << endl;
  }
    
  // process and build new model
  if (!ReadFromURDF(filename.c_str(), built_model, verbose))
  {
    std::cerr << "Error loading model from " << filename << endl;
    abort();
  }
}

bool Model_builder::construct_model (Model* rbdl_model, const ModelPtr& urdf_model, bool verbose) {
    //URDFLinkMap link_map;
    //link_map = urdf_model->links_;
    std::map<std::string, urdf::Link*> link_map;
    for(const auto& links : urdf_model->links_) {
        link_map.insert({links.first, links.second.get()});
    }

    //URDFJointMap joint_map;
    //joint_map = urdf_model->joints_;
    std::map<std::string, urdf::Joint*> joint_map;
    for(const auto& joints : urdf_model->joints_) {
        joint_map.insert({joints.first, joints.second.get()});
    }

	vector<string> joint_names;

	//stack<LinkPtr> link_stack;
    stack<urdf::Link*> link_stack;
	stack<int> joint_index_stack;

	// add the bodies in a depth-first order of the model tree
	link_stack.push(link_map[urdf_model->root_link_->name]);

	// add the root body
	const auto& root = urdf_model->root_link_;
	Vector3d root_inertial_rpy;
	Vector3d root_inertial_position;
	Matrix3d root_inertial_inertia;
	double root_inertial_mass;

	// In the actual implementation is it preferred to NOT use this, and specify the base joint as FLOATING instead
	if (root->inertial) {
		root_inertial_mass = root->inertial->mass;

		root_inertial_position.set (
				root->inertial->origin.position.x,
				root->inertial->origin.position.y,
				root->inertial->origin.position.z);

		root_inertial_inertia(0,0) = root->inertial->ixx;
		root_inertial_inertia(0,1) = root->inertial->ixy;
		root_inertial_inertia(0,2) = root->inertial->ixz;

		root_inertial_inertia(1,0) = root->inertial->ixy;
		root_inertial_inertia(1,1) = root->inertial->iyy;
		root_inertial_inertia(1,2) = root->inertial->iyz;

		root_inertial_inertia(2,0) = root->inertial->ixz;
		root_inertial_inertia(2,1) = root->inertial->iyz;
		root_inertial_inertia(2,2) = root->inertial->izz;

		root->inertial->origin.rotation.getRPY (root_inertial_rpy[0], root_inertial_rpy[1], root_inertial_rpy[2]);

		Body root_link = Body (root_inertial_mass,
				root_inertial_position,
				root_inertial_inertia);
		Joint root_joint = Joint (
				SpatialVector (0., 0., 0., 1., 0., 0.),
				SpatialVector (0., 0., 0., 0., 1., 0.),
				SpatialVector (0., 0., 0., 0., 0., 1.),
				SpatialVector (1., 0., 0., 0., 0., 0.),
				SpatialVector (0., 1., 0., 0., 0., 0.),
				SpatialVector (0., 0., 1., 0., 0., 0.));

		SpatialTransform root_joint_frame = SpatialTransform ();

		if (verbose) {
			cout << "+ Adding Root Body " << endl;
			cout << "  joint frame: " << root_joint_frame << endl;
			cout << "  joint dofs : " << root_joint.mDoFCount << endl;
			for (unsigned int j = 0; j < root_joint.mDoFCount; j++) {
				cout << "    " << j << ": " << root_joint.mJointAxes[j].transpose() << endl;
			}
			cout << "  body inertia: " << endl << root_link.mInertia << endl;
			cout << "  body mass   : " << root_link.mMass << endl;
			cout << "  body name   : " << root->name << endl;
		}

		rbdl_model->AppendBody(root_joint_frame,
				root_joint,
				root_link,
				root->name);
	}

	if (link_stack.top()->child_joints.size() > 0) {
		joint_index_stack.push(0);
	} else {
	    std::cerr << link_stack.top()->child_links.size() << std::endl;
        std::cerr << link_stack.top()->child_joints.size() << std::endl;
        std::cerr << link_stack.top()->name << std::endl;
	}

	while (link_stack.size() > 0) {
		const auto& cur_link = link_stack.top();
		unsigned int joint_idx = joint_index_stack.top();

		if (joint_idx < cur_link->child_joints.size()) {
			const auto& cur_joint = cur_link->child_joints[joint_idx];

			// increment joint index
			joint_index_stack.pop();
			joint_index_stack.push (joint_idx + 1);

			link_stack.push (link_map[cur_joint->child_link_name]);
			joint_index_stack.push(0);

			if (verbose) {
				for (unsigned int i = 1; i < joint_index_stack.size() - 1; i++) {
					cout << "  ";
				}
				cout << "joint '" << cur_joint->name << "' child link '" << link_stack.top()->name << "' type = " << cur_joint->type << endl;
			}

			joint_names.push_back(cur_joint->name);
		} else {
			link_stack.pop();
			joint_index_stack.pop();
		}
	}

	unsigned int j;
	for (j = 0; j < joint_names.size(); j++) {
	        if(verbose)
		  cout << j << ": " << joint_names[j] << endl;
		
		auto& urdf_joint = joint_map[joint_names[j]];
		auto& urdf_parent = link_map[urdf_joint->parent_link_name];
		auto& urdf_child = link_map[urdf_joint->child_link_name];

		// determine where to add the current joint and child body
		unsigned int rbdl_parent_id = 0;

		if (urdf_parent->name != "base_link" && rbdl_model->mBodies.size() >= 1)
			rbdl_parent_id = rbdl_model->GetBodyId (urdf_parent->name.c_str());
		

		if (rbdl_parent_id == std::numeric_limits<unsigned int>::max())
			cerr << "Error while processing joint '" << urdf_joint->name
				<< "': parent link '" << urdf_parent->name
				<< "' could not be found." << endl;

		// create the joint
		Joint rbdl_joint;
		//if the acive joint list is set and the current joint is not in the list of active joints, then this joint is set to FIXED
		if(active_set && !(std::find(active_joints.begin(), active_joints.end(),joint_names[j])!=active_joints.end())) 
		{
		  if(verbose)
		    cout << "Set to fixed : " << j << ", " << joint_names[j] << endl;
		  rbdl_joint = Joint (JointTypeFixed);
		} else{
		  if (urdf_joint->type == urdf::Joint::REVOLUTE || urdf_joint->type == urdf::Joint::CONTINUOUS) {
			  rbdl_joint = Joint (SpatialVector (urdf_joint->axis.x, urdf_joint->axis.y, urdf_joint->axis.z, 0., 0., 0.));
		  } else if (urdf_joint->type == urdf::Joint::PRISMATIC) {
			  rbdl_joint = Joint (SpatialVector (0., 0., 0., urdf_joint->axis.x, urdf_joint->axis.y, urdf_joint->axis.z));
		  } else if (urdf_joint->type == urdf::Joint::FIXED) {
			  rbdl_joint = Joint (JointTypeFixed);
		  } else if (urdf_joint->type == urdf::Joint::FLOATING) {
			  // todo: what order of DoF should be used?
			  if(set_planar_base)
			    rbdl_joint = Joint (
					  SpatialVector (0., 0., 0., 1., 0., 0.),
					  SpatialVector (0., 0., 0., 0., 0., 1.),
					  SpatialVector (0., 1., 0., 0., 0., 0.));
			  else
			    rbdl_joint = Joint (
					  SpatialVector (0., 0., 0., 1., 0., 0.),
					  SpatialVector (0., 0., 0., 0., 1., 0.),
					  SpatialVector (0., 0., 0., 0., 0., 1.),
					  SpatialVector (1., 0., 0., 0., 0., 0.),
					  SpatialVector (0., 1., 0., 0., 0., 0.),
					  SpatialVector (0., 0., 1., 0., 0., 0.));
		  } else if (urdf_joint->type == urdf::Joint::PLANAR) {
			  // todo: which two directions should be used that are perpendicular
			  // to the specified axis?
			  cerr << "Error while processing joint '" << urdf_joint->name << "': planar joints not yet supported!" << endl;
			  return false;
		  }
		}

		// compute the joint transformation
		Vector3d joint_rpy;
		Vector3d joint_translation;
		urdf_joint->parent_to_joint_origin_transform.rotation.getRPY (joint_rpy[0], joint_rpy[1], joint_rpy[2]);
		joint_translation.set (
				urdf_joint->parent_to_joint_origin_transform.position.x,
				urdf_joint->parent_to_joint_origin_transform.position.y,
				urdf_joint->parent_to_joint_origin_transform.position.z
				);
		// if fixed joint angles are set and the current joint is set, then change the orientation of the frame accordingly
		vector<string>::iterator it;
		SpatialTransform Fixed_rot;
		if(fixed_set && (it=std::find(joints_fixed_angle.begin(), joints_fixed_angle.end(),joint_names[j]))!=joints_fixed_angle.end())
		{
		  //set fixed joint angles only if the active joint list is set and so a previous check on fixed joints was made or if the current joint is already defined as fixed in the URDF
		  if(active_set || urdf_joint->type == urdf::Joint::FIXED) 
		  {
		    if(verbose)
		      cout << "Setting angle to fixed joint: " << joint_names[j] << endl;
		    int curr_fix = std::distance(joints_fixed_angle.begin(), it);
		    
// 		    cout << "++++++++++++++++++++++++++++" << urdf_joint->axis.x << "," << urdf_joint->axis.y << "," << urdf_joint->axis.z << endl;
		    // check on which axis the joint is rotating
		    if(abs(urdf_joint->axis.x) == 1)
		      Fixed_rot = Xrot (fixed_angles[curr_fix], Vector3d (1., 0., 0.));
		    else if(abs(urdf_joint->axis.y) == 1)
		      Fixed_rot = Xrot (fixed_angles[curr_fix], Vector3d (0., 1., 0.));
		    else
		      Fixed_rot = Xrot (fixed_angles[curr_fix], Vector3d (0., 0., 1.));
		  }
		  else
		  {
		    cerr << "Impossible to set a fixed angle on a non fixed joint." << endl;
		    abort();
		  }
		}
		SpatialTransform rbdl_joint_frame = 
				      Fixed_rot *
				      Xrot (joint_rpy[0], Vector3d (1., 0., 0.))
				      * Xrot (joint_rpy[1], Vector3d (0., 1., 0.))
				      * Xrot (joint_rpy[2], Vector3d (0., 0., 1.))
				      * Xtrans (Vector3d (joint_translation));

		// assemble the body
		Vector3d link_inertial_position;
		Vector3d link_inertial_rpy;
		Matrix3d link_inertial_inertia = Matrix3d::Zero();
		double link_inertial_mass = 0.;

		// but only if we actually have inertial data
		if (urdf_child->inertial) {

			link_inertial_position.set (
					urdf_child->inertial->origin.position.x,
					urdf_child->inertial->origin.position.y,
					urdf_child->inertial->origin.position.z
					);
			urdf_child->inertial->origin.rotation.getRPY (link_inertial_rpy[0], link_inertial_rpy[1], link_inertial_rpy[2]);
			
			if(set_half_bodies && (it=std::find(half_bodies.begin(), half_bodies.end(),urdf_child->name))!=half_bodies.end())
			{
			  link_inertial_mass = urdf_child->inertial->mass/2;

			  link_inertial_inertia(0,0) = urdf_child->inertial->ixx/2;
			  link_inertial_inertia(0,1) = urdf_child->inertial->ixy/2;
			  link_inertial_inertia(0,2) = urdf_child->inertial->ixz/2;

			  link_inertial_inertia(1,0) = urdf_child->inertial->ixy/2;
			  link_inertial_inertia(1,1) = urdf_child->inertial->iyy/2;
			  link_inertial_inertia(1,2) = urdf_child->inertial->iyz/2;

			  link_inertial_inertia(2,0) = urdf_child->inertial->ixz/2;
			  link_inertial_inertia(2,1) = urdf_child->inertial->iyz/2;
			  link_inertial_inertia(2,2) = urdf_child->inertial->izz/2;
			}
			else{
			  link_inertial_mass = urdf_child->inertial->mass;

			  link_inertial_inertia(0,0) = urdf_child->inertial->ixx;
			  link_inertial_inertia(0,1) = urdf_child->inertial->ixy;
			  link_inertial_inertia(0,2) = urdf_child->inertial->ixz;

			  link_inertial_inertia(1,0) = urdf_child->inertial->ixy;
			  link_inertial_inertia(1,1) = urdf_child->inertial->iyy;
			  link_inertial_inertia(1,2) = urdf_child->inertial->iyz;

			  link_inertial_inertia(2,0) = urdf_child->inertial->ixz;
			  link_inertial_inertia(2,1) = urdf_child->inertial->iyz;
			  link_inertial_inertia(2,2) = urdf_child->inertial->izz;
			}

			if (link_inertial_rpy != Vector3d (0., 0., 0.)) {
				cerr << "Error while processing body '" << urdf_child->name << "': rotation of body frames not yet supported. Please rotate the joint frame instead." << endl;
				return false;
			}
		}

		Body rbdl_body = Body (link_inertial_mass, link_inertial_position, link_inertial_inertia);

		if (verbose) {
			cout << "+ Adding Body " << endl;
			cout << "  parent_id  : " << rbdl_parent_id << endl;
			cout << "  joint frame: " << rbdl_joint_frame << endl;
			cout << "  joint dofs : " << rbdl_joint.mDoFCount << endl;
			for (unsigned int j = 0; j < rbdl_joint.mDoFCount; j++) {
				cout << "    " << j << ": " << rbdl_joint.mJointAxes[j].transpose() << endl;
			}
			cout << "  body inertia: " << endl << rbdl_body.mInertia << endl;
			cout << "  body mass   : " << rbdl_body.mMass << endl;
			cout << "  body name   : " << urdf_child->name << endl;
		}

		if (0 && urdf_joint->type == urdf::Joint::FLOATING) {
			Matrix3d zero_matrix = Matrix3d::Zero();
			Body null_body (0., Vector3d::Zero(3), zero_matrix);
			Joint joint_txtytz(JointTypeTranslationXYZ);
			string trans_body_name = urdf_child->name + "_Translate";
			rbdl_model->AddBody (rbdl_parent_id, rbdl_joint_frame, joint_txtytz, null_body, trans_body_name);

			Joint joint_euler_zyx (JointTypeEulerXYZ);
			rbdl_model->AppendBody (SpatialTransform(), joint_euler_zyx, rbdl_body, urdf_child->name);
		} else {
			rbdl_model->AddBody (rbdl_parent_id, rbdl_joint_frame, rbdl_joint, rbdl_body, urdf_child->name);
		}
	}

	return true;
}

bool Model_builder::ReadFromURDF (const char* filename, Model* rbdl_model,  bool verbose) {
	ifstream model_file (filename);
	if (!model_file) {
		cerr << "Error opening file '" << filename << "'." << endl;
		abort();
	}
	
	// reserve memory for the contents of the file
	string model_xml_string;
	model_file.seekg(0, std::ios::end);
	model_xml_string.reserve(model_file.tellg());
	model_file.seekg(0, std::ios::beg);
	model_xml_string.assign((std::istreambuf_iterator<char>(model_file)), std::istreambuf_iterator<char>());

	model_file.close();
	
	assert(rbdl_model);

	//std::shared_ptr<urdf::ModelInterface> urdf_model = urdf::parseURDF (model_xml_string);
    const auto& urdf_model = urdf::parseURDF (model_xml_string);

    if(!urdf_model) {
        std::cerr << "Error parsing URDF model." << std::endl;
        return false;
    }

	if (!construct_model (rbdl_model, urdf_model, verbose)) {
		cerr << "Error constructing model from urdf file." << endl;
		return false;
	}

	if(gravity_set)
	  rbdl_model->gravity.set (gravity_custom[0],gravity_custom[1],gravity_custom[2]);
	else
	  rbdl_model->gravity.set (0., 0., -9.81);

	return true;
}

template<typename T>
std::string strvec(const T& vec) {
    std::string out{""};
    for(unsigned int i{0}; i < vec.size()-1; ++i){
        out+= std::to_string(vec[i]);
        out += ",";
    }

    out+=std::to_string(vec[vec.size()-1]);
    return out;
}

//T is a vector from eigen
//The elements are emplaced one by one into the `to` vector
template<typename T, typename S>
inline void addVector(T && from, std::vector<S> & to) {
    for(size_t i{0}; i < from.size(); ++i) {
        to.emplace_back(from[i]);
    }
}

template<typename T, typename S>
inline void copyFromEigenMat(T && from, std::vector<std::vector<S>> & to) {
    for(size_t i{0}; i < from.rows(); ++i) {
        std::vector<S> tmp;
        for(size_t j{0}; j < from.cols(); ++j) {
            tmp.emplace_back(from(i,j));
        }
        to.emplace_back(tmp);
    }
}

bool Model_builder::LuaModelWriteToFile(const char* filename, Model* model, bool verbose, bool write_fixed) {
	std::ofstream outfile (filename);
	if (!outfile) {
		std::cerr << "Error opening file " << filename << " for writing!" << std::endl;
		abort();
	}
	double mass{0.0};

	struct Frame{
	    std::vector<std::vector<double>> joint;
	    std::vector<double> joint_frame_r;
	    std::vector<std::vector<double>> joint_frame_E;
	    std::string joint_type;
	    std::string parent;
	    std::string name;
	    double mass;
	    std::vector<double> com;
	    std::vector<std::vector<double>> inertia;
	};

	std::vector<Frame> frames;

	bool have_virtual_joint_frame = false;
	unsigned int virtual_joint_index = 0;
	SpatialTransform virtual_joint_transform = SpatialTransform ();
	SpatialVector virtual_joint_axes[6];

	for (size_t i{1}; i < model->mBodies.size(); i++) {
		unsigned int parent_id = model->lambda[i];
		while (model->mBodies[parent_id].mIsVirtual)
			parent_id = model->lambda[parent_id];

		Frame frame = Frame();

		std::string parent_name = model->GetBodyName(parent_id);
        std::string body_name = model->GetBodyName(i);

		if(body_name == "") continue; //skip unnamed parts
		mass += model->mBodies[i].mMass;
		//std::cout << body_name << ":\t\t\t" << model->mBodies[i].mMass << std::endl; //output mass

		if (model->mBodies[i].mIsVirtual) {
			if (!have_virtual_joint_frame) {
				have_virtual_joint_frame = true;
				virtual_joint_transform = model->X_T[i];
				virtual_joint_index = 1;
				virtual_joint_axes[0] = model->mJoints[i].mJointAxes[0];
				virtual_joint_axes[1].setZero();
				virtual_joint_axes[2].setZero();
				virtual_joint_axes[3].setZero();
				virtual_joint_axes[4].setZero();
				virtual_joint_axes[5].setZero();
				continue;
			} else {
				virtual_joint_axes[virtual_joint_index] = model->mJoints[i].mJointAxes[0];
				virtual_joint_index ++;
			}
		} else {
			if (have_virtual_joint_frame) {
				for (unsigned int j = 0; j < virtual_joint_index; j++) {
					std::vector<double> tmp;
					for(size_t ii{0}; ii < virtual_joint_axes[j].size(); ++ii)
					    tmp.push_back(virtual_joint_axes[j][ii]);
					frame.joint.emplace_back(tmp);
				}

				std::vector<double> tmp;
                addVector(model->mJoints[i].mJointAxes[0], tmp);
                frame.joint.emplace_back(tmp);

				if (virtual_joint_transform.r.squaredNorm() != 0.) {
                    addVector(virtual_joint_transform.r, frame.joint_frame_r);
				}
				if (virtual_joint_transform.E != Matrix3d::Identity(3,3)) {
					copyFromEigenMat(virtual_joint_transform.E, frame.joint_frame_E);
				}
				have_virtual_joint_frame = false;
			} else {
                std::vector<double> tmp;
				switch (model->mJoints[i].mJointType) {
					case JointTypeRevolute:
					case JointTypePrismatic:
					case JointTypeRevoluteX:
					case JointTypeRevoluteY:
					case JointTypeRevoluteZ:
                        addVector(model->mJoints[i].mJointAxes[0], tmp);
                        frame.joint.emplace_back(tmp);
                        frame.joint_type = "\"JointTypeRevolute\"";
						break;
					case JointTypeTranslationXYZ:
						frame.joint_type = "\"JointTypeTranslationXYZ\""; break;
					case JointTypeSpherical:
                        frame.joint_type = "\"JointTypeSpherical\""; break;
					case JointTypeEulerXYZ:
                        frame.joint_type = "\"JointTypeEulerXYZ\""; break;
					case JointTypeEulerZYX:
                        frame.joint_type = "\"JointTypeEulerZYX\""; break;
					case JointTypeEulerYXZ:
                        frame.joint_type = "\"JointTypeEulerYXZ\""; break;
					default:
						cerr << "Invalid joint type " << model->mJoints[i].mJointType << " for joint with id " << i << endl;
						abort();
				}
			}
		}

		if (parent_name != "") {
			frame.parent = parent_name;
		}
		if (body_name != "") {
			frame.name = body_name;
		}

		if (parent_name == "ROOT" && body_name != "base_link") {
		    frame.parent = "base_link";
		}

		if (model->X_T[i].r.squaredNorm() != 0.) {
            addVector(model->X_T[i].r, frame.joint_frame_r);
		}
		if (model->X_T[i].E != Matrix3d::Identity (3,3) ) {
            copyFromEigenMat(model->X_T[i].E, frame.joint_frame_E);
		}
		
		frame.mass = model->mBodies[i].mMass;
        addVector(model->mBodies[i].mCenterOfMass, frame.com);
        copyFromEigenMat(model->mBodies[i].mInertia, frame.inertia);

		frames.emplace_back(frame);
	}
	
	if(write_fixed) { // export also fixed bodies
	  for(size_t i{0}; i < model->mFixedBodies.size(); ++i) {
	      Frame frame = Frame();
		  unsigned int parent_id = model->mFixedBodies[i].mMovableParent;
		  std::string parent_name = model->GetBodyName(parent_id);
		  std::string body_name = model->GetBodyName(i+model->fixed_body_discriminator);

		  if(verbose)
		    cout << "Fixed body: " << i << ", parent name: " << parent_name << endl;
		  if (parent_name != "") {
			  frame.parent = parent_name;
		  }
		  if (body_name != "") {
			  if(verbose)
			    cout << "body name: " << body_name << endl;
			  frame.name = body_name;
		  }

		  if (model->mFixedBodies[i].mParentTransform.r.squaredNorm() != 0.) {
              addVector(model->mFixedBodies[i].mParentTransform.r, frame.joint_frame_r);
			  if(verbose)
			      std::cout << "joint_frame r: " << model->mFixedBodies[i].mParentTransform.r;
		  }
		  if (model->mFixedBodies[i].mParentTransform.E != Matrix3d::Identity (3,3) ) {
              copyFromEigenMat(model->mFixedBodies[i].mParentTransform.E, frame.joint_frame_E);
              if(verbose)
                  std::cout << "joint_frame E: " << model->mFixedBodies[i].mParentTransform.E;
		  }
	  }
	}

    outfile << "model = {" << std::endl;
    outfile << "gravity = {" << strvec(model->gravity) << "}," << std::endl;
    outfile << "frames = {" << std::endl;

	for(const auto& f: frames) {
        outfile << "{" << std::endl; //frame open
        outfile << "name = \"" << f.name << "\"," << std::endl;
        outfile << "parent = \"" << f.parent << "\"," << std::endl;
        outfile << "body = {" << std::endl; //body open
            outfile << "mass = " << f.mass << ", " << std::endl;
            outfile << "com = {" << strvec(f.com) << "}, " << std::endl;
            outfile << "inertia = { " << std::endl;
            outfile << "{" << strvec(f.inertia[0]) << "}," << std::endl;
            outfile << "{" << strvec(f.inertia[1]) << "}," << std::endl;
            outfile << "{" << strvec(f.inertia[2]) << "}}" << std::endl;
        outfile << "}," << std::endl; //body close

        if(f.joint.size() > 0) {
            outfile << "joint = {" << std::endl; //joint open
            for(const auto& v:f.joint) {
                outfile << "{" << strvec(v) << "}," << std::endl;
            }
            outfile << "}," << std::endl; //joint close
        }
//        else if (f.joint_type != "") {
//            if(f.joint_type != "") {
//                outfile << "joint =f.joint_type << std::endl;
//            }
//        }

        if(f.joint_frame_E.size() > 0 || f.joint_frame_r.size() > 0) {
            outfile << "joint_frame = {" << std::endl; //joint_frame open
            if (f.joint_frame_r.size() > 0) {
                outfile << "r = {" << strvec(f.joint_frame_r) << "}," << std::endl;
            }
            if (f.joint_frame_E.size() > 0) {
                outfile << "E = {{" << strvec(f.joint_frame_E[0]) << "}," << std::endl;
                outfile << "{" << strvec(f.joint_frame_E[1]) << "}," << std::endl;
                outfile << "{" << strvec(f.joint_frame_E[2]) << "}}," << std::endl;
            }
            outfile << "}," << std::endl; //joint_frame close
        }
        outfile << "}," << std::endl; //frame close
	}

    outfile << "}}" << std::endl;
    outfile << "return model"<< std::endl;

	outfile.close();

	//std::cout << "totalmass:\t\t\t" << mass << std::endl; //output total mass

	return true;
}

bool Model_builder::UrdfModelWriteToFile(const char* filename, Model* model, bool verbose, bool write_fixed) {
    std::ofstream outfile (filename);
    if (!outfile) {
        std::cerr << "Error opening file " << filename << " for writing!" << std::endl;
        abort();
    }
    double mass{0.0};

    struct Frame{
        std::vector<std::vector<double>> joint;
        std::vector<double> joint_frame_r;
        std::vector<std::vector<double>> joint_frame_E;
        std::string joint_type;
        std::string parent;
        std::string name;
        double mass;
        std::vector<double> com;
        std::vector<std::vector<double>> inertia;
    };

    std::vector<Frame> frames;

    bool have_virtual_joint_frame = false;
    unsigned int virtual_joint_index = 0;
    SpatialTransform virtual_joint_transform = SpatialTransform ();
    SpatialVector virtual_joint_axes[6];

    for (size_t i{1}; i < model->mBodies.size(); i++) {
        unsigned int parent_id = model->lambda[i];
        while (model->mBodies[parent_id].mIsVirtual)
            parent_id = model->lambda[parent_id];

        Frame frame = Frame();

        std::string parent_name = model->GetBodyName(parent_id);
        std::string body_name = model->GetBodyName(i);

        if(body_name == "") continue; //skip unnamed parts
        mass += model->mBodies[i].mMass;
        //std::cout << body_name << ":\t\t\t" << model->mBodies[i].mMass << std::endl;

        if (model->mBodies[i].mIsVirtual) {
            if (!have_virtual_joint_frame) {
                have_virtual_joint_frame = true;
                virtual_joint_transform = model->X_T[i];
                virtual_joint_index = 1;
                virtual_joint_axes[0] = model->mJoints[i].mJointAxes[0];
                virtual_joint_axes[1].setZero();
                virtual_joint_axes[2].setZero();
                virtual_joint_axes[3].setZero();
                virtual_joint_axes[4].setZero();
                virtual_joint_axes[5].setZero();
                continue;
            } else {
                virtual_joint_axes[virtual_joint_index] = model->mJoints[i].mJointAxes[0];
                virtual_joint_index ++;
            }
        } else {
            if (have_virtual_joint_frame) {
                for (unsigned int j = 0; j < virtual_joint_index; j++) {
                    std::vector<double> tmp;
                    for(size_t ii{0}; ii < virtual_joint_axes[j].size(); ++ii)
                        tmp.push_back(virtual_joint_axes[j][ii]);
                    frame.joint.emplace_back(tmp);
                }

                std::vector<double> tmp;
                addVector(model->mJoints[i].mJointAxes[0], tmp);
                frame.joint.emplace_back(tmp);

                if (virtual_joint_transform.r.squaredNorm() != 0.) {
                    addVector(virtual_joint_transform.r, frame.joint_frame_r);
                }
                if (virtual_joint_transform.E != Matrix3d::Identity(3,3)) {
                    copyFromEigenMat(virtual_joint_transform.E, frame.joint_frame_E);
                }
                have_virtual_joint_frame = false;
            } else {
                std::vector<double> tmp;
                switch (model->mJoints[i].mJointType) {
                case JointTypeRevolute:
                case JointTypePrismatic:
                case JointTypeRevoluteX:
                case JointTypeRevoluteY:
                case JointTypeRevoluteZ:
                    addVector(model->mJoints[i].mJointAxes[0], tmp);
                    frame.joint.emplace_back(tmp);
                    frame.joint_type = "\"JointTypeRevolute\"";
                    break;
                case JointTypeTranslationXYZ:
                    frame.joint_type = "\"JointTypeTranslationXYZ\""; break;
                case JointTypeSpherical:
                    frame.joint_type = "\"JointTypeSpherical\""; break;
                case JointTypeEulerXYZ:
                    frame.joint_type = "\"JointTypeEulerXYZ\""; break;
                case JointTypeEulerZYX:
                    frame.joint_type = "\"JointTypeEulerZYX\""; break;
                case JointTypeEulerYXZ:
                    frame.joint_type = "\"JointTypeEulerYXZ\""; break;
                default:
                    cerr << "Invalid joint type " << model->mJoints[i].mJointType << " for joint with id " << i << endl;
                    abort();
                }
            }
        }

        if (parent_name != "") {
            frame.parent = parent_name;
        }
        if (body_name != "") {
            frame.name = body_name;
        }

        if (model->X_T[i].r.squaredNorm() != 0.) {
            addVector(model->X_T[i].r, frame.joint_frame_r);
        }
        if (model->X_T[i].E != Matrix3d::Identity (3,3) ) {
            copyFromEigenMat(model->X_T[i].E, frame.joint_frame_E);
        }

        frame.mass = model->mBodies[i].mMass;
        addVector(model->mBodies[i].mCenterOfMass, frame.com);
        copyFromEigenMat(model->mBodies[i].mInertia, frame.inertia);

        frames.emplace_back(frame);
    }

    if(write_fixed) { // export also fixed bodies
        for(size_t i{0}; i < model->mFixedBodies.size(); ++i) {
            Frame frame = Frame();
            unsigned int parent_id = model->mFixedBodies[i].mMovableParent;
            std::string parent_name = model->GetBodyName(parent_id);
            std::string body_name = model->GetBodyName(i+model->fixed_body_discriminator);

            if(verbose)
                cout << "Fixed body: " << i << ", parent name: " << parent_name << endl;
            if (parent_name != "") {
                frame.parent = parent_name;
            }
            if (body_name != "") {
                if(verbose)
                    cout << "body name: " << body_name << endl;
                frame.name = body_name;
            }

            if (model->mFixedBodies[i].mParentTransform.r.squaredNorm() != 0.) {
                addVector(model->mFixedBodies[i].mParentTransform.r, frame.joint_frame_r);
                if(verbose)
                    std::cout << "joint_frame r: " << model->mFixedBodies[i].mParentTransform.r;
            }
            if (model->mFixedBodies[i].mParentTransform.E != Matrix3d::Identity (3,3) ) {
                copyFromEigenMat(model->mFixedBodies[i].mParentTransform.E, frame.joint_frame_E);
                if(verbose)
                    std::cout << "joint_frame E: " << model->mFixedBodies[i].mParentTransform.E;
            }
        }
    }

    ////////////////////////////////////////////////////////////////
    urdf::ModelInterface mymodel;

    mymodel.name_ = "conversion_lua_urdf";

    //for we create and add links
    for(const auto& f: frames) {
        urdf::LinkSharedPtr mylink;
        mylink.reset(new urdf::Link);

        std::string parent_name = f.parent;
        std::string body_name = f.name;

        mylink->name = body_name;

        //Inertial
        mylink->inertial.reset(new urdf::Inertial);

        mylink->inertial->mass = f.mass;
        mylink->inertial->ixx = f.inertia[0][0];
        mylink->inertial->ixy = f.inertia[0][1];
        mylink->inertial->ixz = f.inertia[0][2];
        mylink->inertial->iyy = f.inertia[1][1];
        mylink->inertial->iyz = f.inertia[1][2];
        mylink->inertial->izz = f.inertia[2][2];

        urdf::Pose inertialpose;
        inertialpose.position.x = f.com[0];
        inertialpose.position.y = f.com[1];
        inertialpose.position.z = f.com[2];

        mylink->inertial->origin = inertialpose;

        mymodel.links_.insert(make_pair(mylink->name, mylink));
    }

    //in the second pass, we create and add joints and also connect the links together
    for(auto& f: frames) {
        urdf::JointSharedPtr myjoint;
        myjoint.reset(new urdf::Joint);

        std::string parent_name = f.parent;
        std::string body_name = f.name;

        if(parent_name == "ROOT") continue; //skip root

        if(f.joint_type == "\"JointTypeTranslationXYZ\"" ||
                f.joint_type == "\"JointTypeSpherical\"" ||
                f.joint_type == "\"JointTypeEulerXYZ\"" ||
                f.joint_type == "\"JointTypeEulerZYX\"" ||
                f.joint_type == "\"JointTypeEulerYXZ\"" ||
                f.joint_type == "\"JointTypeRevolute\"") {
            myjoint->type = urdf::Joint::REVOLUTE;;

            myjoint->limits.reset(new urdf::JointLimits);
            //myjoint->limits->
        } else if(f.joint_type == "\"JointTypeFloatingBase\"")
            myjoint->type = urdf::Joint::FLOATING;
        else if(f.joint_type == "\"JointTypeFixed\"")
            myjoint->type = urdf::Joint::FIXED;
        else if(f.joint_type == "\"JointTypePrismatic\"")
            myjoint->type = urdf::Joint::PRISMATIC;
        else
            myjoint->type = urdf::Joint::UNKNOWN;

        myjoint->parent_link_name = parent_name;
        myjoint->child_link_name = body_name;

        myjoint->name = body_name + "_joint";

        if(f.joint_frame_r.size() > 0) {
            urdf::Pose mypose;
            mypose.position.x = f.joint_frame_r[0];
            mypose.position.y = f.joint_frame_r[1];
            mypose.position.z = f.joint_frame_r[2];

            myjoint->parent_to_joint_origin_transform.position = mypose.position;
        }

        if(f.joint_frame_E.size() > 0) {

            double ay1, ay2, az1, az2, ax1, ax2;
            if (f.joint_frame_E[0][2] == -1) { //gimbal lock
                az1 = std::atan2(f.joint_frame_E[0][2], f.joint_frame_E[0][2]);
                az2 = az1;
                ax1 = 0.0;
                ax1 = 0.0;
            } else if (f.joint_frame_E[0][2] == 1) { //gimbal lock
                az1 = std::atan2(f.joint_frame_E[0][2], f.joint_frame_E[0][2]);
                az2 = az1;
                ax1 = 0.0;
                ax1 = 0.0;
            } else {

                double ay1 = -std::asin(f.joint_frame_E[0][2]);
                double ay2 = M_PI - ay1;

                double az1 = std::atan2(f.joint_frame_E[0][1] / ay1, f.joint_frame_E[0][0] / ay1);
                double az2 = std::atan2(f.joint_frame_E[0][1] / ay2, f.joint_frame_E[0][0] / ay2);

                double ax1 = std::atan2(f.joint_frame_E[1][2] / ay1, f.joint_frame_E[2][2] / ay1);
                double ax2 = std::atan2(f.joint_frame_E[1][2] / ay2, f.joint_frame_E[2][2] / ay2);
            }
            urdf::Pose mypose;

            if(std::max({ax2, ay2, az2}) > M_PI || std::min({ax2, ay2, az2}) < -M_PI) { //use ax1, ay1, az1
                mypose.rotation.setFromRPY(ax1, ay1, az1);
            } else { //use ax2, ay2, az2
                mypose.rotation.setFromRPY(ax2, ay2, az2);
            }

            myjoint->parent_to_joint_origin_transform.rotation = mypose.rotation;
        }

        if(f.joint.size() > 0) {
                myjoint->axis.x = f.joint[0][0];
                myjoint->axis.y = f.joint[0][1];
                myjoint->axis.z = f.joint[0][2];
        }

        if(mymodel.joints_.find(myjoint->name) == mymodel.joints_.end()) {
            mymodel.joints_.insert(make_pair(myjoint->name, myjoint));
        } else {
            std::cerr << "Joint " << myjoint->name << " already exists." << std::endl;

            return false;
        }

        urdf::LinkSharedPtr mylink;
        mylink.reset(new urdf::Link);

        urdf::CollisionSharedPtr mycollision;
        mycollision.reset(new urdf::Collision);

        urdf::LinkSharedPtr myparentlink;
        myparentlink.reset(new urdf::Link);

        mymodel.getLink(body_name, mylink);
        mymodel.getLink(parent_name, myparentlink);

        mylink->setParent(myparentlink);
        //std::cout << "j" << std::endl;
    }

    auto res = urdf::exportURDF(mymodel);

    res->SaveFile(filename);

    return true;
}