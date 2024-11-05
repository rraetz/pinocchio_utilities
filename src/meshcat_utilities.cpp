#include "meshcat_utilities.h"
#include "logging.h"

#include <MeshcatCpp/Material.h>
#include <MeshcatCpp/Meshcat.h>
#include <MeshcatCpp/Shape.h>
#include "meshcat_utilities.h"

#include "pinocchio/multibody/geometry.hpp"
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/algorithm/geometry.hpp>

enum GeometryType 
{
    BOX,
    SPHERE,
    CYLINDER,
    MESH
};


MeshcatCpp::MatrixView<double> SE3_to_matrix_view(const pinocchio::SE3 &transform) 
{
  static Eigen::Matrix4d matrix;
  matrix = transform.toHomogeneousMatrix();
  constexpr auto order = MeshcatCpp::MatrixStorageOrdering::ColumnMajor;
  return MeshcatCpp::make_matrix_view(matrix.data(), 4, 4, order);
}


const GeometryType detect_geometry_type(const pinocchio::GeometryObject& object) 
{
    std::shared_ptr<hpp::fcl::CollisionGeometry> geometry = object.geometry;
    if (object.meshPath != "") {
        return GeometryType::MESH;
    } else if (std::dynamic_pointer_cast<hpp::fcl::Box>(geometry)) {
        return GeometryType::BOX;
    } else if (std::dynamic_pointer_cast<hpp::fcl::Sphere>(geometry)) {
        return GeometryType::SPHERE;
    } else if (std::dynamic_pointer_cast<hpp::fcl::Cylinder>(geometry)) {
        return GeometryType::CYLINDER;
    } else {
        throw std::runtime_error("Geometry type not supported");
    }
}


void add_geometry_model_to_meshcat(MeshcatCpp::Meshcat& meshcat, const pinocchio::GeometryModel& geom_model)
{
  static MeshcatCpp::Material m;
  for (const auto& object : geom_model.geometryObjects) 
  {
    const GeometryType geometry_type = detect_geometry_type(object);
    LOG_DEBUG << "Detected geometry type: " << geometry_type;

    const int red = rand() % 255;
    const int green = rand() % 255;
    const int blue = rand() % 255;
    m.set_color(red, green, blue);

    if (geometry_type == GeometryType::BOX)
    {
      auto box = *(std::dynamic_pointer_cast<hpp::fcl::Box>(object.geometry));
      const double x = box.halfSide[0]*2;
      const double y = box.halfSide[1]*2;
      const double z = box.halfSide[2]*2;
      meshcat.set_object(object.name, MeshcatCpp::Box(x, y, z), m);
      LOG_DEBUG << "Box with name " << object.name << " added to Meshcat";
    }

    else if (geometry_type == GeometryType::SPHERE)
    {
      auto sphere = *(std::dynamic_pointer_cast<hpp::fcl::Sphere>(object.geometry));
      const double radius = sphere.radius;
      meshcat.set_object(object.name, MeshcatCpp::Sphere(radius), m);
      LOG_DEBUG << "Sphere with name " << object.name << " added to Meshcat";
    }

    else if (geometry_type == GeometryType::CYLINDER)
    {
      // FIXME: The orientation is not correct. In Meshcat, the cylinder is oriented along the y-axis
      auto cylinder = *(std::dynamic_pointer_cast<hpp::fcl::Cylinder>(object.geometry));
      const double radius = cylinder.radius;
      const double length = cylinder.halfLength*2;
      meshcat.set_object(object.name, MeshcatCpp::Cylinder(radius, length), m);
      LOG_DEBUG << "Cylinder with name " << object.name << " added to Meshcat";
    }
      
    else if (geometry_type == GeometryType::MESH)
    {
      auto mesh_path = object.meshPath;
      if (mesh_path.find(".STL") != std::string::npos) {
        LOG_WARNING << "The mesh file of " << mesh_path << 
          " ending is capitalized. Please use lowercase instead.";
      }
      meshcat.set_object(object.name, MeshcatCpp::Mesh(mesh_path), m);
      LOG_DEBUG << "Mesh with name " << object.name << " added to Meshcat";
    }
  }
}


void update_meshcat_geometry_poses(MeshcatCpp::Meshcat& meshcat, const pinocchio::GeometryModel& geom_model, const pinocchio::GeometryData& geom_data)
{
    for (pinocchio::GeomIndex id=0; id < geom_model.ngeoms; ++id)
    {
        const auto& object = geom_model.geometryObjects[id];
        auto pose = geom_data.oMg[id];
        meshcat.set_transform(object.name, SE3_to_matrix_view(pose));
    }
}
   
