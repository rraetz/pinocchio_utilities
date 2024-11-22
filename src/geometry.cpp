#include "geometry.h"
#include "logging.h"

void add_box(pinocchio::GeometryModel& geom_model, const std::string& name, const pinocchio::SE3& pose, double x, double y, double z) 
{
    auto geometry = std::make_shared<hpp::fcl::Box>(x, y, z);
    geom_model.addGeometryObject(pinocchio::GeometryObject(name, 0, 0, pose, geometry));
    LOG_DEBUG << "Geometry BOX added: " << name;
}


void add_sphere(pinocchio::GeometryModel& geom_model, const std::string& name, const pinocchio::SE3& pose, double radius) 
{
    auto geometry = std::make_shared<hpp::fcl::Sphere>(radius);
    geom_model.addGeometryObject(pinocchio::GeometryObject(name, 0, 0, pose, geometry));
    LOG_DEBUG << "Geometry SPHERE added: " << name;
}


void add_cylinder(pinocchio::GeometryModel& geom_model, const std::string& name, const pinocchio::SE3& pose, double radius, double length) 
{
    auto geometry = std::make_shared<hpp::fcl::Cylinder>(radius, length);
    geom_model.addGeometryObject(pinocchio::GeometryObject(name, 0, 0, pose, geometry));
    LOG_DEBUG << "Geometry CYLINDER added: " << name;
}


bool has_collision(const pinocchio::GeometryModel& geom_model, const pinocchio::GeometryData& geom_data)
{
    bool return_value = false;
    for (size_t k = 0; k < geom_model.collisionPairs.size(); ++k)
    {
        const pinocchio::CollisionPair & cp = geom_model.collisionPairs[k];
        const hpp::fcl::CollisionResult & cr = geom_data.collisionResults[k];
        if (cr.isCollision())
        {
            return_value = true;
            const auto &name1 = geom_model.geometryObjects[cp.first].name;
            const auto &name2 = geom_model.geometryObjects[cp.second].name;
            // LOG_DEBUG << "Collision: " << name1 << " <-> " << name2;
        }
    }
    return return_value;
}


void append_collision_pair_mapping(pinocchio::GeometryModel &combined_geom_model, const pinocchio::GeometryModel &source_geom_model)
{
    // Assumes that the new geometry models are already merged!!!
    const auto new_size = combined_geom_model.ngeoms; 
    const auto source_size = source_geom_model.ngeoms;
    const auto old_size = new_size - source_size;
    const auto n_old_collision_pairs = combined_geom_model.collisionPairs.size() - source_geom_model.collisionPairs.size();

    combined_geom_model.collisionPairMapping.conservativeResize(new_size, new_size);
    combined_geom_model.collisionPairMapping.bottomLeftCorner(source_size, old_size).fill(-1);
    combined_geom_model.collisionPairMapping.topRightCorner(old_size, source_size).fill(-1);
    combined_geom_model.collisionPairMapping.bottomRightCorner(source_size, source_size).fill(-1);

    // Map new collision pairs from the source model to the new combined model
    for (size_t pair_index = 0; pair_index < source_geom_model.collisionPairs.size(); ++pair_index)
    {
        const auto &pair = source_geom_model.collisionPairs[pair_index];
        const auto first = pair.first + old_size;
        const auto second = pair.second + old_size;
        combined_geom_model.collisionPairMapping(first, second) = pair_index + n_old_collision_pairs;
        combined_geom_model.collisionPairMapping(second, first) = pair_index + n_old_collision_pairs;
    }
}


void compute_distances(const pinocchio::GeometryModel &geom_model, pinocchio::GeometryData &geom_data)
{
    pinocchio::computeDistances(geom_model, geom_data);
    for (size_t cp_index = 0; cp_index < geom_model.collisionPairs.size(); ++cp_index)
    {
        const auto &result = geom_data.distanceResults[cp_index];
        const pinocchio::CollisionPair & cp = geom_model.collisionPairs[cp_index];
        const auto &name1 = geom_model.geometryObjects[cp.first].name;
        const auto &name2 = geom_model.geometryObjects[cp.second].name;
        const double distance = result.min_distance;
        const auto &p1 = result.nearest_points[0];
        const auto &p2 = result.nearest_points[1];
        const auto position_delta = p1-p2;
        LOG_DEBUG << "Distance: " << name1 << " <-> " << name2 << ": " << distance;
        LOG_DEBUG << "Position delta: " << position_delta;
    }
}