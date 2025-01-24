package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.robot.Constants.SpeedConstants;

public class DriveToPoseUtil {

        // TODO: constraints for these pids
        // pids for driving to a pose
        private static final double DRIVE_TO_POSE_XY_P = 0.1;
        private static final double DRIVE_TO_POSE_XY_D = 0.01;
        private static final ProfiledPIDController DRIVE_TO_POSE_XY_PID = new ProfiledPIDController(
                        DRIVE_TO_POSE_XY_P, 0, DRIVE_TO_POSE_XY_D, null);

        private static final double DRIVE_TO_POSE_THETA_P = 0.1;
        private static final double DRIVE_TO_POSE_THETA_D = 0.01;
        private static final ProfiledPIDController DRIVE_TO_POSE_THETA_PID = new ProfiledPIDController(
                        DRIVE_TO_POSE_THETA_P, 0, DRIVE_TO_POSE_THETA_D, null);

        // tolerances
        private static final Distance XY_ALIGN_TOLERANCE = Inches.of(1);
        private static final Angle THETA_ALIGN_TOLERANCE = Degrees.of(5);

        public static Transform2d getDriveToPoseVelocities(Pose2d robotPose, Pose2d goalPose) {
                Transform2d transformToGoal = robotPose.minus(goalPose);
                LinearVelocity xDesired = MetersPerSecond.of(DRIVE_TO_POSE_XY_PID.calculate(transformToGoal.getX(), 0)
                                * SpeedConstants.DRIVETRAIN_MAX_SPEED_MPS);
                LinearVelocity yDesired = MetersPerSecond.of(DRIVE_TO_POSE_XY_PID.calculate(transformToGoal.getY(), 0)
                                * SpeedConstants.DRIVETRAIN_MAX_SPEED_MPS);
                AngularVelocity thetaDesired = RotationsPerSecond
                                .of(DRIVE_TO_POSE_THETA_PID.calculate(transformToGoal.getRotation().getRadians())
                                                * SpeedConstants.DRIVETRAIN_MAX_ANGULAR_SPEED_RPS);
                if (transformToGoal.getX() < XY_ALIGN_TOLERANCE.in(Meters)) {
                        xDesired = MetersPerSecond.of(0);
                }
                if (transformToGoal.getY() < XY_ALIGN_TOLERANCE.in(Meters)) {
                        yDesired = MetersPerSecond.of(0);
                }
                if (transformToGoal.getRotation().getRadians() < THETA_ALIGN_TOLERANCE.in(Radians)) {
                        thetaDesired = RotationsPerSecond.of(0);
                }

                // packaging as a Transform2d because we don't have access to gyro here so
                // cannot do ChassiSpeeds
                return new Transform2d(xDesired.in(MetersPerSecond),
                                yDesired.in(MetersPerSecond),
                                new Rotation2d(thetaDesired.in(RadiansPerSecond)));
        }

        public static AngularVelocity getAlignmentRotRate(Pose2d robotPose, Pose2d goalPose) {
                Rotation2d rotationToGoal = robotPose.minus(goalPose).getRotation();
                AngularVelocity thetaDesired = RotationsPerSecond.of(
                                DRIVE_TO_POSE_THETA_PID.calculate(rotationToGoal.getRotations(), 0));
                if (rotationToGoal.getDegrees() < THETA_ALIGN_TOLERANCE.in(Degrees)) {
                        thetaDesired = RotationsPerSecond.of(0);
                }
                return thetaDesired;
        }
}
