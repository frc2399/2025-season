package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DriveToPoseUtil {
        // profiled pid controllers for driving to a pose and related constants
        private static final double DRIVE_TO_POSE_XY_P = 10;
        private static final double DRIVE_TO_POSE_XY_D = 0.0;
        private static final PIDController driveToPoseXPid = new PIDController(DRIVE_TO_POSE_XY_P, 0,
                        DRIVE_TO_POSE_XY_D);
        private static final PIDController driveToPoseYPid = new PIDController(DRIVE_TO_POSE_XY_P, 0,
                        DRIVE_TO_POSE_XY_D);

        private static final double DRIVE_TO_POSE_THETA_P = 3.5; // radians per second per radian of error
        private static final double DRIVE_TO_POSE_THETA_D = 0.0;
        private static final PIDController driveToPoseThetaAltPid = new PIDController(DRIVE_TO_POSE_THETA_P, 0,
                        DRIVE_TO_POSE_THETA_D);
        // Pose2d automatically wraps to -180 to 180 degrees. if this changes, these
        // values need to change, too.
        private static final Angle DRIVE_TO_POSE_MIN_INPUT = Degrees.of(-180);
        private static final Angle DRIVE_TO_POSE_MAX_INPUT = Degrees.of(180);

        // this is called a static block. it is here because this util has no state
        // dependencies. this pid controller should always have continuous input
        // wrapping, and the way to make that happen in a static fashion is a static
        // block
        static {
                driveToPoseThetaAltPid.enableContinuousInput(
                                DRIVE_TO_POSE_MIN_INPUT.in(Degrees),
                                DRIVE_TO_POSE_MAX_INPUT.in(Degrees));
        }

        // tolerance constants
        private static final Distance XY_ALIGN_TOLERANCE = Inches.of(0.25);
        private static final Angle THETA_ALIGN_TOLERANCE = Degrees.of(1);

        private static final Distance NEXT_ACTION_TRIGGER_DIST = Feet.of(1); // the distance at which the next command
                                                                             // in any automated control scheme will run

        public static Supplier<ChassisSpeeds> getDriveToPoseVelocities(Supplier<Pose2d> robotPose,
                        Supplier<Pose2d> goalPose) {

                                // calculate desired robot-relative velocities
                LinearVelocity xDesired = MetersPerSecond
                                .of(driveToPoseXPid.calculate(robotPose.get().getX(), goalPose.get().getX()));
                LinearVelocity yDesired = MetersPerSecond
                                .of(driveToPoseYPid.calculate(robotPose.get().getY(), goalPose.get().getY()));
                AngularVelocity thetaDesired = RadiansPerSecond
                                .of(driveToPoseThetaAltPid.calculate(robotPose.get().getRotation().getRadians(),
                                                goalPose.get().getRotation().getRadians()));

                double xError = robotPose.get().getX() - goalPose.get().getX();
                SmartDashboard.putNumber("vision/xError_drivetopose", xError);
                double yError = robotPose.get().getY() - goalPose.get().getY();
                SmartDashboard.putNumber("vision/yError_drivetopose", yError);
                Angle thetaError = Radians.of(
                                robotPose.get().getRotation().getRadians() - goalPose.get().getRotation().getRadians());

                // tolerance checking
                if (Math.abs(xError) < XY_ALIGN_TOLERANCE.in(Meters)) {
                        xDesired = MetersPerSecond.of(0);
                }
                if (Math.abs(yError) < XY_ALIGN_TOLERANCE.in(Meters)) {
                        yDesired = MetersPerSecond.of(0);
                }
                if (Math.abs(thetaError.in(Degrees)) < THETA_ALIGN_TOLERANCE.in(Degrees)) {
                        thetaDesired = RadiansPerSecond.of(0);
                }

                // when the requested theta approaches 0, it is hard to get the robot to
                // actually move, and this ks, where s is static, helps ensure that it reaches
                // its goal
                if (Math.abs(thetaDesired.in(RadiansPerSecond)) > 0
                                && Math.abs(thetaDesired.in(RadiansPerSecond)) < 0.1) {
                        thetaDesired = RadiansPerSecond.of(Math.copySign(0.1, thetaDesired.in(RadiansPerSecond)));
                }

                SmartDashboard.putNumber("vision/xDesiredVelocity_drivetopose", xDesired.in(MetersPerSecond));
                SmartDashboard.putNumber("vision/yDesiredVelocity_drivetopose", yDesired.in(MetersPerSecond));

                ChassisSpeeds alignmentSpeeds = new ChassisSpeeds(xDesired.in(MetersPerSecond),
                                yDesired.in(MetersPerSecond),
                                thetaDesired.in(RadiansPerSecond));

                return () -> alignmentSpeeds;
        }

        public static Boolean poseWithinToleranceForNextAction(Supplier<Pose2d> robotPose, Supplier<Pose2d> goalPose) {
                Distance distToGoal = Meters.of(robotPose.get().getTranslation().getDistance(goalPose.get().getTranslation()));
                return distToGoal.in(Meters) <= NEXT_ACTION_TRIGGER_DIST.in(Meters);
        }
}
