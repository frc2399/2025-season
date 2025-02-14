package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.robot.Constants.SpeedConstants;

public class DriveToPoseUtil {
        // pids for driving to a pose
        private static final double DRIVE_TO_POSE_XY_P = 1.2;
        private static final double DRIVE_TO_POSE_XY_D = 0.0;
        private static final LinearVelocity MAX_VELOCITY_DRIVE_TO_POSE = MetersPerSecond.of(1);
        private static final LinearAcceleration MAX_ACCELERATION_DRIVE_TO_POSE = MetersPerSecondPerSecond.of(0.5);
        private static final ProfiledPIDController DRIVE_TO_POSE_XY_PID = new ProfiledPIDController(
                        DRIVE_TO_POSE_XY_P, 0, DRIVE_TO_POSE_XY_D,
                        new Constraints(MAX_VELOCITY_DRIVE_TO_POSE.in(MetersPerSecond),
                                        MAX_ACCELERATION_DRIVE_TO_POSE.in(MetersPerSecondPerSecond)));

        private static final double DRIVE_TO_POSE_THETA_P = 1.8;
        private static final double DRIVE_TO_POSE_THETA_D = 0.0;
        private static final AngularVelocity MAX_ANGULAR_VELOCITY_DRIVE_TO_POSE = DegreesPerSecond.of(45);
        private static final AngularAcceleration MAX_ANGULAR_ACCELERATION_DRIVE_TO_POSE = DegreesPerSecondPerSecond
                        .of(10);
        private static final ProfiledPIDController DRIVE_TO_POSE_THETA_PID = new ProfiledPIDController(
                        DRIVE_TO_POSE_THETA_P, 0, DRIVE_TO_POSE_THETA_D,
                        new Constraints(MAX_ANGULAR_VELOCITY_DRIVE_TO_POSE.in(RadiansPerSecond),
                                        MAX_ANGULAR_ACCELERATION_DRIVE_TO_POSE.in(RadiansPerSecondPerSecond)));

        // tolerances
        private static final Distance XY_ALIGN_TOLERANCE = Inches.of(1);
        private static final Angle THETA_ALIGN_TOLERANCE = Degrees.of(3);
        
        //filtering
        private static final Distance XY_MAX_ALIGN_DISTANCE = Meters.of(4);

        public static Supplier<Transform2d> getDriveToPoseVelocities(Supplier<Pose2d> robotPose,
                        Supplier<Pose2d> goalPose) {
                if (robotPose.get() == null) {
                        Transform2d nullReturn = new Transform2d(0, 0, new Rotation2d(0));
                        return () -> nullReturn;
                }
                Transform2d transformToGoal = robotPose.get().minus(goalPose.get());
                LinearVelocity xDesired = MetersPerSecond.of(DRIVE_TO_POSE_XY_PID.calculate(transformToGoal.getX(), 0)
                                * SpeedConstants.DRIVETRAIN_MAX_SPEED_MPS);
                LinearVelocity yDesired = MetersPerSecond.of(DRIVE_TO_POSE_XY_PID.calculate(transformToGoal.getY(), 0)
                                * SpeedConstants.DRIVETRAIN_MAX_SPEED_MPS);
                AngularVelocity thetaDesired = RotationsPerSecond
                                .of(DRIVE_TO_POSE_THETA_PID.calculate(transformToGoal.getRotation().getRadians())
                                                * SpeedConstants.DRIVETRAIN_MAX_ANGULAR_SPEED_RPS);
                double xToGoal = transformToGoal.getX();
                double yToGoal = transformToGoal.getY();
                if (Math.hypot(xToGoal, yToGoal) > XY_MAX_ALIGN_DISTANCE.in(Meters)) {
                        xDesired = MetersPerSecond.of(0);
                        yDesired = MetersPerSecond.of(0);
                }
                if (Math.abs(xToGoal) < XY_ALIGN_TOLERANCE.in(Meters)) {
                        xDesired = MetersPerSecond.of(0);
                }
                if (Math.abs(yToGoal) < XY_ALIGN_TOLERANCE.in(Meters)) {
                        yDesired = MetersPerSecond.of(0);
                }
                if (transformToGoal.getRotation().getRadians() < THETA_ALIGN_TOLERANCE.in(Radians)) {
                        thetaDesired = RotationsPerSecond.of(0);
                }

                // packaging as a Transform2d because we don't have access to gyro here so
                // cannot do ChassiSpeeds
                Transform2d alignmentSpeeds = new Transform2d(-xDesired.in(MetersPerSecond),
                                -yDesired.in(MetersPerSecond),
                                new Rotation2d(-thetaDesired.in(RadiansPerSecond)));
                return () -> alignmentSpeeds;
        }

        public static AngularVelocity getAlignmentRotRate(Pose2d robotPose, Supplier<Pose2d> goalPose) {
                Rotation2d rotationToGoal = robotPose.minus(goalPose.get()).getRotation();
                AngularVelocity thetaDesired = RotationsPerSecond.of(
                                DRIVE_TO_POSE_THETA_PID.calculate(rotationToGoal.getRotations(), 0));
                if (rotationToGoal.getDegrees() < THETA_ALIGN_TOLERANCE.in(Degrees)) {
                        thetaDesired = RotationsPerSecond.of(0);
                }
                return thetaDesired;
        }
}
