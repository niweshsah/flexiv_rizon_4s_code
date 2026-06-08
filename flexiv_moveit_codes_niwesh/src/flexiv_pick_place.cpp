/**
 * @file flexiv_pick_place.cpp
 * @brief MoveIt 2 Pick-and-Place sequence for Flexiv Rizon 4s.
 * * Demonstrates safe Cartesian planning, state validation, custom placeholder
 * gripper actions, and robust exception handling.
 */

#include <memory>
#include <thread>
#include <vector>
#include <string>
#include <chrono>
#include <stdexcept>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

using namespace std::chrono_literals;

// =============================================================================
// Configuration Structure
// =============================================================================
struct TaskConfig {
    double pick_x, pick_y, pick_z;
    double place_x, place_y, place_z;
    double pick_wait, place_wait;
};

// =============================================================================
// Flexiv Pick & Place Application Class
// =============================================================================
class FlexivPickAndPlace
{
public:
    FlexivPickAndPlace(rclcpp::Node::SharedPtr node, const TaskConfig& config)
    : node_(node), logger_(node->get_logger()), config_(config), 
      move_group_(node, "rizon_arm") // Default Flexiv arm planning group
    {
        // Motion Safety & Limits
        RCLCPP_INFO(logger_, "Configuring MoveIt 2 safety parameters...");
        move_group_.setMaxVelocityScalingFactor(0.2);     // 20% max velocity for safety
        move_group_.setMaxAccelerationScalingFactor(0.2); // 20% max acceleration
        move_group_.setPlanningTime(10.0);                // Max time allowed for the planner
        move_group_.setNumPlanningAttempts(5);            // Number of attempts if planning fails
        
        // Define downward-facing orientation (Tool Z-axis pointing down)
        // Adjust RPY based on specific tool frame. M_PI on X typically flips Z down.
        tf2::Quaternion q;
        q.setRPY(M_PI, 0.0, 0.0);
        downward_orientation_.x = q.x();
        downward_orientation_.y = q.y();
        downward_orientation_.z = q.z();
        downward_orientation_.w = q.w();
    }

    /**
     * @brief Main sequence execution.
     */
    bool runSequence()
    {
        try {
            RCLCPP_INFO(logger_, "=== Starting Flexiv Pick and Place Sequence ===");

            // 1. Record Home Pose (Current Joint State)
            RCLCPP_INFO(logger_, "Recording Home Pose...");
            std::vector<double> home_joint_values = move_group_.getCurrentJointValues();
            if (home_joint_values.empty()) {
                RCLCPP_ERROR(logger_, "Failed to retrieve current joint states! Aborting.");
                return false;
            }

            // 2. Move to Pick Position
            geometry_msgs::msg::Pose pick_pose;
            pick_pose.orientation = downward_orientation_;
            pick_pose.position.x = config_.pick_x;
            pick_pose.position.y = config_.pick_y;
            pick_pose.position.z = config_.pick_z;

            RCLCPP_INFO(logger_, "Moving to Pick Position: [%.3f, %.3f, %.3f]", 
                        pick_pose.position.x, pick_pose.position.y, pick_pose.position.z);
            if (!planAndExecutePose(pick_pose, "Pick Pose")) return abortSequence();

            // 3. Perform Pick Action
            if (!performPickAction()) return abortSequence();

            // 4. Move to Place Position
            geometry_msgs::msg::Pose place_pose;
            place_pose.orientation = downward_orientation_;
            place_pose.position.x = config_.place_x;
            place_pose.position.y = config_.place_y;
            place_pose.position.z = config_.place_z;

            RCLCPP_INFO(logger_, "Moving to Place Position: [%.3f, %.3f, %.3f]", 
                        place_pose.position.x, place_pose.position.y, place_pose.position.z);
            if (!planAndExecutePose(place_pose, "Place Pose")) return abortSequence();

            // 5. Perform Place Action
            if (!performPlaceAction()) return abortSequence();

            // 6. Return to Home Pose
            RCLCPP_INFO(logger_, "Returning safely to Home Pose...");
            if (!planAndExecuteJoints(home_joint_values, "Home Pose")) return abortSequence();

            RCLCPP_INFO(logger_, "=== Sequence Completed Successfully ===");
            return true;

        } catch (const std::exception& ex) {
            RCLCPP_FATAL(logger_, "Runtime Exception during sequence: %s", ex.what());
            return false;
        }
    }

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Logger logger_;
    TaskConfig config_;
    moveit::planning_interface::MoveGroupInterface move_group_;
    geometry_msgs::msg::Quaternion downward_orientation_;

    // =========================================================================
    // Helper Functions
    // =========================================================================

    bool planAndExecutePose(const geometry_msgs::msg::Pose& target_pose, const std::string& pose_name)
    {
        move_group_.setPoseTarget(target_pose);
        moveit::planning_interface::MoveGroupInterface::Plan my_plan;

        RCLCPP_INFO(logger_, "Planning path to %s...", pose_name.c_str());
        bool success = (move_group_.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
        
        if (!success) {
            RCLCPP_ERROR(logger_, "Planning failed for %s. Target may be unreachable or in collision.", pose_name.c_str());
            return false;
        }

        RCLCPP_INFO(logger_, "Executing trajectory to %s...", pose_name.c_str());
        success = (move_group_.execute(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
        
        if (!success) {
            RCLCPP_ERROR(logger_, "Execution failed for %s! Hardware issue or collision detected.", pose_name.c_str());
            return false;
        }

        return true;
    }

    bool planAndExecuteJoints(const std::vector<double>& target_joints, const std::string& pose_name)
    {
        move_group_.setJointValueTarget(target_joints);
        moveit::planning_interface::MoveGroupInterface::Plan my_plan;

        RCLCPP_INFO(logger_, "Planning path to %s (Joint Space)...", pose_name.c_str());
        bool success = (move_group_.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
        
        if (!success) {
            RCLCPP_ERROR(logger_, "Joint Planning failed for %s.", pose_name.c_str());
            return false;
        }

        RCLCPP_INFO(logger_, "Executing trajectory to %s...", pose_name.c_str());
        success = (move_group_.execute(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
        
        if (!success) {
            RCLCPP_ERROR(logger_, "Joint Execution failed for %s!", pose_name.c_str());
            return false;
        }

        return true;
    }

    bool abortSequence()
    {
        RCLCPP_ERROR(logger_, "!!! SEQUENCE ABORTED due to safety or planning failure !!!");
        // Optional: Implement a fallback recovery behavior here.
        return false;
    }

    // =========================================================================
    // Placeholder Gripper Actions
    // =========================================================================

    bool performPickAction()
    {
        RCLCPP_INFO(logger_, "[ACTION] Executing Pick... (Closing Gripper)");
        // Explicitly cast the double (seconds) into std::chrono::nanoseconds
        auto wait_duration = std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::duration<double>(config_.pick_wait)
        );
        rclcpp::sleep_for(wait_duration);
        RCLCPP_INFO(logger_, "[ACTION] Pick completed.");
        return true;
    }

    bool performPlaceAction()
    {
        RCLCPP_INFO(logger_, "[ACTION] Executing Place... (Opening Gripper)");
        // Explicitly cast the double (seconds) into std::chrono::nanoseconds
        auto wait_duration = std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::duration<double>(config_.place_wait)
        );
        rclcpp::sleep_for(wait_duration);
        RCLCPP_INFO(logger_, "[ACTION] Place completed.");
        return true;
    }
};

// =============================================================================
// CLI Parser & Main Entry
// =============================================================================

void printUsage() {
    std::cout << "Usage: flexiv_pick_place <px> <py> <pz> <plx> <ply> <plz> <pick_wait> <place_wait>\n"
              << "Arguments:\n"
              << "  px, py, pz     : Pick position coordinates (meters)\n"
              << "  plx, ply, plz  : Place position coordinates (meters)\n"
              << "  pick_wait      : Time to wait at pick pose (seconds)\n"
              << "  place_wait     : Time to wait at place pose (seconds)\n"
              << "Example:\n"
              << "  ros2 run my_pkg flexiv_pick_place 0.5 0.2 0.3 0.5 -0.2 0.3 2.0 1.5\n";
}

int main(int argc, char** argv)
{
    if (argc != 9) {
        printUsage();
        return 1;
    }

    TaskConfig config;
    try {
        config.pick_x = std::stod(argv[1]);
        config.pick_y = std::stod(argv[2]);
        config.pick_z = std::stod(argv[3]);
        config.place_x = std::stod(argv[4]);
        config.place_y = std::stod(argv[5]);
        config.place_z = std::stod(argv[6]);
        config.pick_wait = std::stod(argv[7]);
        config.place_wait = std::stod(argv[8]);
    } catch (const std::exception& e) {
        std::cerr << "Error parsing arguments: " << e.what() << "\nEnsure all inputs are valid numbers.\n";
        printUsage();
        return 1;
    }

    rclcpp::init(argc, argv);

    // MoveIt 2 relies heavily on the parameter server, this option is required
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    auto node = rclcpp::Node::make_shared("flexiv_pick_and_place_node", node_options);

    // MoveIt 2 action clients need a dedicated spinner thread to function correctly
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    std::thread spinner([&executor]() { executor.spin(); });

    // Initialize application and run
    FlexivPickAndPlace app(node, config);
    app.runSequence();

    // Clean shutdown
    rclcpp::shutdown();
    spinner.join();

    return 0;
}