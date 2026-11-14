#include "ros2_livox/livox_ode_multiray_shape.h"
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <sdf/sdf.hh>
#include <cmath>
#include <iostream>
#include <stdexcept>

int main(int argc, char ** argv)
{
  gazebo::setupServer(argc, argv);
  auto world = gazebo::loadWorld("worlds/empty.world");
  if (!world) {throw std::runtime_error("Cannot load isolated Gazebo test world");}
  auto link = world->ModelByName("ground_plane")->GetLink();
  auto collision = world->Physics()->CreateCollision("ray", link);
  {
    boost::shared_ptr<gazebo::physics::LivoxOdeMultiRayShape> shape(
      new gazebo::physics::LivoxOdeMultiRayShape(collision));
    collision->SetShape(shape);
    sdf::SDFPtr config(new sdf::SDF);
    sdf::init(config);
    const std::string xml = R"(
      <sdf version="1.6"><model name="sensor"><link name="link">
      <sensor name="lidar" type="ray"><ray><scan>
      <horizontal><samples>2</samples><min_angle>0</min_angle><max_angle>1</max_angle></horizontal>
      <vertical><samples>1</samples><min_angle>0</min_angle><max_angle>0</max_angle></vertical>
      </scan><range><min>0.1</min><max>200</max><resolution>0.001</resolution></range>
      </ray></sensor></link></model></sdf>)";
    if (!sdf::readString(xml, config)) {throw std::runtime_error("Invalid sensor test SDF");}
    shape->Load(config->Root()->GetElement("model")->GetElement("link")->GetElement("sensor"));
    shape->Init();
    const auto equal = [](double actual, double expected) {
        if (std::abs(actual - expected) > 1e-8) {
          throw std::runtime_error("LiDAR range differs from configured physical distance");
        }
      };
    equal(shape->GetMinRange(), .1);
    equal(shape->GetMaxRange(), 200);
    if (!shape->RayShapes().empty()) {throw std::runtime_error("Regular scan grid was retained");}
    shape->AddRay({.1,0,0}, {200,0,0});
    // The ODE hit distance is measured from the ray's near-range start.
    shape->RayShapes().front()->SetLength(1.9);
    equal(shape->GetRange(0), 2.0);
    shape->RayShapes().front()->SetLength(199.9);
    equal(shape->GetRange(0), 200.0);
    std::cout << "Range initialization and hit/no-hit distance checks passed\n";
  }
  world.reset();
  gazebo::shutdown();
}

