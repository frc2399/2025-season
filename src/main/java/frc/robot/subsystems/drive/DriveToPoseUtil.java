package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.robot.Constants.SpeedConstants;

public class DriveToPoseUtil {
    // pids for driving to a pose
    private static final double DRIVE_TO_POSE_XY_P = 0.1;
    private static final double DRIVE_TO_POSE_XY_D = 0.01;
    private static final ProfiledPIDController DRIVE_TO_POSE_XY_PID = new ProfiledPIDController(
            DRIVE_TO_POSE_XY_P, 0, DRIVE_TO_POSE_XY_D, null);

    private static final double DRIVE_TO_POSE_THETA_P = 0.1;
    private static final double DRIVE_TO_POSE_THETA_D = 0.01;
    private static final ProfiledPIDController DRIVE_TO_POSE_THETA_PID = new ProfiledPIDController(
            DRIVE_TO_POSE_THETA_P, 0, DRIVE_TO_POSE_THETA_D, null);

    public static Transform2d getDriveToPoseVelocities(Pose2d goal, Pose2d robotPose) {
        Transform2d transformToGoal = robotPose.minus(goal);
        LinearVelocity xDesired = MetersPerSecond.of(DRIVE_TO_POSE_XY_PID.calculate(transformToGoal.getX(), 0)
                * SpeedConstants.DRIVETRAIN_MAX_SPEED_MPS);
        LinearVelocity yDesired = MetersPerSecond.of(DRIVE_TO_POSE_XY_PID.calculate(transformToGoal.getY(), 0)
                * SpeedConstants.DRIVETRAIN_MAX_SPEED_MPS);
        AngularVelocity thetaDesired = RotationsPerSecond
                .of(DRIVE_TO_POSE_THETA_PID.calculate(transformToGoal.getRotation().getRadians())
                        * SpeedConstants.DRIVETRAIN_MAX_ANGULAR_SPEED_RPS);
        return new Transform2d(xDesired.in(MetersPerSecond), yDesired.in(MetersPerSecond),
                new Rotation2d(thetaDesired.in(RotationsPerSecond)));
    }
}
