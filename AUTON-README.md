# How 2 auton good

## What you should know

We've got Megatag2 localization working on this year's robot, so we can start from anywhere vaguely near the start position of the path, as long as the robot was rebooted/zeroed facing the red alliance wall

## Prereqs

- Choreo, latest (v2025.0.3 or later)
- Pathplanner, latest (v2025.2.2 or later)
- ability to deploy code to the robot


## Workflow

1. Open AdvantageScope, plot the following on the "Odometry" view
  - `VisionPoseEstimator/` - a pose2d representing the last pose seen by the camera
  - `DriveSubsystem/EstimatedPose` - a pose2d representing the pose-estimator's current pose
  - `/PathPlanner/targetPose` - where PathPlanner is trying to get the robot
2. Boot the robot (or load code) facing the red alliance wall. Make sure the `VisionPoseEstimator` pose matches where the robot is on the field.
3. Open the FRC driver station and Elastic on the driver station laptop, select an auton using the AutonPicker widget
4. Open Choreo, Pathplanner, and VSCode on your programming laptop.
5. Run an auton, observe. Does the robot follow the `EstimatedPose` and do the actions you intend?
  - if yes, you are done
  - if no, pick the section below that best describes your problem

### Robot does not do the actions I want it to do, but goes to the right place

1. Go into pathplanner, tweak sequential and parallel groups until you are satisfied with the result.
2. For ??? reasons, you might need a 0s wait after a drive

### Robot does not go where I want it to go, and the `VisionPoseEstimator` pose is lost somewhere behind the `targetPose`

1. Can the Limelight (`http://limelight.local:5801`) see an apriltag? is it the right apriltag?
2. If no, open choreo. Find a way to change the path so that the robot can see a tag for more of it.
3. Make sure to click "Generate path"
4. pathplanner will auto-update, redeploy the code when the robot is oriented correctly (facing red alliance wall)

### Robot Does not go where I want it to go, poses all match

1. Tweak the path in choreo, nudging the robot a little closer/further/whatever from where it ends up
2. Make sure to click "Generate path"
3. pathplanner will auto-update, redeploy the code when the robot is oriented correctly (facing red alliance wall)

