#include <ros/ros.h>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/rendering/rendering.hh>
#include <gazebo/common/common.hh>
#include <algorithm>
#include <random>
#include <iostream>

namespace gazebo
{
    class RandomizeAprilTags : public WorldPlugin
    {
        public:
        // Constructor
        RandomizeAprilTags() : WorldPlugin()
        {
            this->renderConnection = event::Events::ConnectPreRender(
                std::bind(&RandomizeAprilTags::OnRender, this));
        }

        // Load function that is called when the plugin is inserted
        void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf)
        {
            ROS_INFO("----------- RandomizeAprilTags Plugin Loaded -----------");

            // Ensure ROS is initialized
            if (!ros::isInitialized())
            {
                ROS_FATAL_STREAM("ROS node for Gazebo has not been initialized.");
                return;
            }

            // Initialize APRIL tag selection
            std::vector<int> available_apriltags = {0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15};
            std::random_device rd;
            std::mt19937 g(rd());
            std::shuffle(available_apriltags.begin(), available_apriltags.end(), g);
            this->selected_apriltags.assign(available_apriltags.begin(), available_apriltags.begin() + 8);

            // Store world pointer
            this->_world = _world;
        }

        // This function is called every render cycle
        void OnRender()
        {
            if (!this->scene)
            {
                // Try to get the scene
                this->scene = rendering::get_scene();
                if (!this->scene)
                {
                    ROS_WARN("Scene is not ready yet, waiting...");
                    return;
                }
            }

            // If we reach here, the scene is available
            for (int i = 1; i <= 8; i++)
            {
                std::string model_name = "poly_" + std::to_string(i);
                physics::ModelPtr model = _world->ModelByName(model_name);

                if (model)
                {
                    std::string link_name = "tag" + std::to_string(i);
                    physics::LinkPtr link = model->GetLink(link_name);

                    if (link)
                    {
                        // Construct the full visual name: model::link::visual
                        std::string visual_name = model->GetScopedName() + "::" + link_name + "::visual";
                        rendering::VisualPtr visual = this->scene->GetVisual(visual_name);

                        if (visual)
                        {
                            // Change material script
                            std::string new_tag_material = "material/artag" + std::to_string(this->selected_apriltags[i - 1]);
                            visual->SetMaterial(new_tag_material);
                            ROS_INFO_STREAM("Changed AprilTag for " << model_name << " to: " << new_tag_material);
                        }
                        else
                        {
                            ROS_WARN_STREAM("Visual not found for " << visual_name);
                        }
                    }
                    else
                    {
                        ROS_WARN_STREAM("Link not found for " << link_name);
                    }
                }
                else
                {
                    ROS_WARN_STREAM("Model not found for " << model_name);
                }
            }

            // Disconnect the rendering connection after updating visuals
            this->renderConnection.reset();
        }

        private:
        physics::WorldPtr _world;
        rendering::ScenePtr scene;
        std::vector<int> selected_apriltags;
        event::ConnectionPtr renderConnection;
    };

    // Register this plugin with Gazebo
    GZ_REGISTER_WORLD_PLUGIN(RandomizeAprilTags)
}