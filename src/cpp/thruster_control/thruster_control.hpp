/*
 * takes joystick command + depth control command and
 * outputs thruster command for ROV
 * space to be sent to thruster mappings
 */

#include <Eigen/Dense>
#include <Eigen/Geometry>

Eigen::Isometry3d genThrusterCommand(
    Eigen::AngleAxisd measuredPose,
    Eigen::Isometry3d joystickCommand,
    float depthControlCommand)
{
    // Rotation from local coordinates to world coordinates
    Eigen::Matrix3d R = measuredPose.toRotationMatrix();

    // 1. Convert joystick translation to world coordinates
    Eigen::Vector3d worldCommand =
        R * joystickCommand.translation();

    // 2. Remove the vertical component
    worldCommand.z() = 0.0;

    // 3. Add the depth controller command as the vertical component
    worldCommand.z() = depthControlCommand;

    // 4. Convert back to local coordinates
    Eigen::Vector3d localCommand =
        R.transpose() * worldCommand;

    // 5. Return the resulting isometry
    Eigen::Isometry3d result = joystickCommand;
    result.translation() = localCommand;

    return result;
}
