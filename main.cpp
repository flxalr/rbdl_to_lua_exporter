#include <iostream>
#include <string>

#include <rbdl/rbdl.h>
#include <rbdl/rbdl_utils.h>

#include <rbdl/addons/luamodel/luamodel.h>

#include "Model_builder.h"

#define RBDL_ENABLE_LOGGING

using namespace RigidBodyDynamics;

bool hasEnding (std::string const &fullString, std::string const &ending) {
    if (fullString.length() >= ending.length()) {
        return (0 == fullString.compare (fullString.length() - ending.length(), ending.length(), ending));
    } else {
        return false;
    }
}

int main(int argc, char* argv[]) {

    std::string exportfilename{"model_export"};
    if(argc < 2 || argc > 3) {
        std::cerr << "usage: " << argv[0] << " <filename> [true|t|1]" << std::endl;
        std::cerr << "\tFirst argument (required) is the filename of the model file which should be imported." << std::endl;
        std::cerr << "\tSupported fileformats are: *.lua and *.urdf" << std::endl;
        std::cerr << "\tExample: ./" << argv[0] << " my_robot_file.urdf" << std::endl;
        std::cerr << "\tThe output will be written to the file: " << exportfilename << std::endl;
        std::cerr << std::endl;
        std::cerr << "\tSecond argument (optional) if fixed bodies should be exported too. Default is false." << std::endl;
        return -1;
    }

    std::string filename{argv[1]};
    bool urdfile{false};


    if (hasEnding(filename, ".lua")) {}
    else if (hasEnding(filename, ".urdf")) {
        urdfile = true;
    } else {
        std::cerr << "Only *.lua or *.urdf files are supported" << std::endl;
        return -1;
    }

    bool export_fixed_bodies{false};

    if(argc == 3) {
        std::string arg2{argv[2]};
        if(arg2 == "t" || arg2 == "true" || arg2 == "1") {
            export_fixed_bodies = true;
        } else {
            std::cerr << "Unsupported argument after filename: " << argv[2] << std::endl;
            std::cerr << "Expected: [true|t|1]" << std::endl;
            return -1;
        }
    }

    Model model;
    Model_builder builder(filename);

    if(urdfile) {
        exportfilename += std::string{".lua"};

        builder.build_model(&model,false);
        builder.LuaModelWriteToFile(exportfilename.c_str(), &model, false, export_fixed_bodies);
    } else { //lua file
        exportfilename += std::string{".urdf"};
        if (!Addons::LuaModelReadFromFile (filename.c_str(), &model, false)) {
            std::cerr << "Error loading model " << filename << std::endl;
            return -1;
         }

        std::cerr << "lua dof: " << model.dof_count << std::endl;

        builder.UrdfModelWriteToFile(exportfilename.c_str(), &model, false, export_fixed_bodies);
    }

    //builder.LuaModelWriteToFile(exportfilename.c_str(), &model, false, export_fixed_bodies);

    std::cout << "Model exported to " << exportfilename << "." << std::endl;

    return 0;
}