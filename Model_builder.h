#ifndef MODEL_BUILDER
#define MODEL_BUILDER

#include <vector>
#include <memory>
#include <map>
#include <string>

#include <rbdl/rbdl.h>

#include <rbdl/addons/luamodel/luamodel.h>
#include <rbdl/addons/luamodel/luatables.h>

#include <urdf_model/model.h>
#include <urdf_parser/urdf_parser.h>

#include <urdf_exception/exception.h>

#define MAX_DOF 100
    
typedef std::shared_ptr<urdf::Link> LinkPtr;
typedef std::shared_ptr<urdf::Joint> JointPtr;
typedef std::shared_ptr<urdf::ModelInterface> ModelPtr;

typedef std::vector<LinkPtr> URDFLinkVector;
typedef std::vector<JointPtr> URDFJointVector;
typedef std::map<std::string, LinkPtr > URDFLinkMap;
typedef std::map<std::string, JointPtr > URDFJointMap;

/**
 * This class allows to build an RBDL model starting from an URDF file.
 * It is possible to set a list of active joints, all the joints that are not in this list will be set automatically to FIXED, independently from their specification in the URDF.
 * It is NOT possible to set a joint as active if in the URDF it is not.
 * It is also possible to set a list of angles at which a certain joint will be fixed. This is possible only for joints that are active in the URDF file and only of type REVOLUTE (for now).
 * The gravity can be changed using the funcion setGravity, otherwise the default gravity (0,0,-9.81) will be used.
 * The built model can be exported as lua model script, where is it possible to specify whether the fixed joints should be exported or not.
 * NOTE: please remind that in RBDL when a joint is declared as fixed, the child body attached to it will be automatically merged with its movable parent body.
 */

#include <rbdl/addons/luamodel/luatables.h>

class Model_builder
{
  public:
	Model_builder(std::string model_file);
	~Model_builder();
	
	void setActiveJoints(std::vector<std::string> user_joints);
	void setFixedAngles(std::vector<std::string> joints, std::vector<double> angles); //check that the joints are not in the active joints
	void setHalfBodies(std::vector< std::string > bodies);
	void build_model(RigidBodyDynamics::Model* built_model, bool verbose);
	void setGravity(RigidBodyDynamics::Math::Vector3d g);
	bool LuaModelWriteToFile(const char* filename, RigidBodyDynamics::Model* model, bool verbose = false, bool write_fixed = true);
    bool UrdfModelWriteToFile(const char* filename, RigidBodyDynamics::Model* model, bool verbose = false, bool write_fixed = true);
	void setPlanarBase();
  
  private:
	
	bool ReadFromURDF(const char* filename, RigidBodyDynamics::Model* rbdl_model, bool verbose = false);
	bool construct_model(RigidBodyDynamics::Model* rbdl_model, const ModelPtr& urdf_model, bool verbose);
    
// 	RigidBodyDynamics::Model model;
	std::string filename;
	std::vector<std::string> active_joints; // if this list is set, then ALL the joints that are NOT in the active joints will be set to FIXED
	std::vector<std::string> joints_fixed_angle;
	std::vector<double> fixed_angles;
	std::vector<std::string> half_bodies;
	bool active_set;
	bool fixed_set;
	bool gravity_set; //if not set, use default 0,0,-9.81
	bool set_half_bodies;
	bool set_planar_base;
	RigidBodyDynamics::Math::Vector3d gravity_custom;
};

#endif