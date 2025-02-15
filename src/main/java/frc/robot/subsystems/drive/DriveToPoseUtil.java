package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;

public class DriveToPoseUtil {
        // pids for driving to a pose
        private static final double DRIVE_TO_POSE_XY_P = 3.75;
        private static final double DRIVE_TO_POSE_XY_D = 0.0;
        private static final LinearVelocity MAX_VELOCITY_DRIVE_TO_POSE = MetersPerSecond.of(1);
        private static final LinearAcceleration MAX_ACCELERATION_DRIVE_TO_POSE = MetersPerSecondPerSecond.of(0.5);
        private static final ProfiledPIDController driveToPoseXYPid = new ProfiledPIDController(
                        DRIVE_TO_POSE_XY_P, 0, DRIVE_TO_POSE_XY_D,
                        new Constraints(MAX_VELOCITY_DRIVE_TO_POSE.in(MetersPerSecond),
                                        MAX_ACCELERATION_DRIVE_TO_POSE.in(MetersPerSecondPerSecond)));
        private static final ProfiledPIDController altXyPid = new ProfiledPIDController(
                        DRIVE_TO_POSE_XY_P, 0, DRIVE_TO_POSE_XY_D,
                        new Constraints(MAX_VELOCITY_DRIVE_TO_POSE.in(MetersPerSecond),
                                        MAX_ACCELERATION_DRIVE_TO_POSE.in(MetersPerSecondPerSecond)));
        
        private static final double DRIVE_TO_POSE_THETA_P = 2.5; // radians per second per radian of error
        private static final double DRIVE_TO_POSE_THETA_D = 0.0;
        private static final AngularVelocity MAX_ANGULAR_VELOCITY_DRIVE_TO_POSE = DegreesPerSecond.of(45);
        private static final AngularAcceleration MAX_ANGULAR_ACCELERATION_DRIVE_TO_POSE = DegreesPerSecondPerSecond
                        .of(10);
        // as of 2025, our gyro wraps -180 to 180. if this changes, these values need to
        // change, too.
        private static final Angle DRIVE_TO_POSE_MIN_INPUT = Degrees.of(-180);
        private static final Angle DRIVE_TO_POSE_MAX_INPUT = Degrees.of(180);
        private static ProfiledPIDController driveToPoseThetaPid = new ProfiledPIDController(
                        DRIVE_TO_POSE_THETA_P, 0, DRIVE_TO_POSE_THETA_D,
                        new Constraints(MAX_ANGULAR_VELOCITY_DRIVE_TO_POSE.in(RadiansPerSecond),
                                        MAX_ANGULAR_ACCELERATION_DRIVE_TO_POSE.in(RadiansPerSecondPerSecond)));
        // this is called a static block. it is here because as of right now, this util
        // has no state dependencies. this pid controller should always have continuous
        // input wrapping, and the way to make that run in a static fashion is this
        // static block
        static {
                driveToPoseThetaPid.enableContinuousInput(
                                DRIVE_TO_POSE_MIN_INPUT.in(Degrees),
                                DRIVE_TO_POSE_MAX_INPUT.in(Degrees));
        }

        // tolerances
        private static final Distance XY_ALIGN_TOLERANCE = Inches.of(0.5);
        private static final Angle THETA_ALIGN_TOLERANCE = Degrees.of(1);

        // filtering
        private static final Distance XY_MAX_ALIGN_DISTANCE = Meters.of(3);
        private static final Angle THETA_MAX_ALIGN_ANGLE = Degrees.of(90);

        public static Supplier<Transform2d> getDriveToPoseVelocities(Supplier<Pose2d> robotPose,
                        Supplier<Pose2d> goalPose) {
                if (robotPose.get() == null) {
                        Transform2d nullReturn = new Transform2d(0, 0, new Rotation2d(0));
                        return () -> nullReturn;
                }
                Transform2d transformToGoal = goalPose.get().minus(robotPose.get());
                LinearVelocity xDesired = MetersPerSecond.of(driveToPoseXYPid.calculate(transformToGoal.getX(), 0));
                LinearVelocity yDesired = MetersPerSecond.of(driveToPoseXYPid.calculate(transformToGoal.getY(), 0));
                AngularVelocity thetaDesired = RadiansPerSecond
                                .of(driveToPoseThetaPid.calculate(transformToGoal.getRotation().getRadians(), 0));
                double xToGoal = transformToGoal.getX();
                double yToGoal = transformToGoal.getY();
                Angle thetaToGoal = Degrees.of(transformToGoal.getRotation().getDegrees());
                
                // filtering - if we're off by too much, it doesn't move. this keeps the robot
                // from doing anything too drastic, especially in case of odometry failures.
                if (Math.hypot(xToGoal, yToGoal) > XY_MAX_ALIGN_DISTANCE.in(Meters) || 
                thetaToGoal.in(Degrees) > THETA_MAX_ALIGN_ANGLE.in(Degrees)) {
                        xDesired = MetersPerSecond.of(0);
                        yDesired = MetersPerSecond.of(0);
                        thetaDesired = RadiansPerSecond.of(0);
                }

                // tolerance checking
                if (Math.abs(xToGoal) < XY_ALIGN_TOLERANCE.in(Meters)) {
                        xDesired = MetersPerSecond.of(0);
                }
                if (Math.abs(yToGoal) < XY_ALIGN_TOLERANCE.in(Meters)) {
                        yDesired = MetersPerSecond.of(0);
                }
                if (Math.abs(thetaToGoal.in(Degrees)) < THETA_ALIGN_TOLERANCE.in(Degrees)) {
                        thetaDesired = RadiansPerSecond.of(0);
                }

                // if the requested theta rotation is too small, make it bigger! (unless it was
                // zeroed out above) (ks, where s = static)
                if (Math.abs(thetaDesired.in(RadiansPerSecond)) > 0
                                && Math.abs(thetaDesired.in(RadiansPerSecond)) < 0.1) {
                        thetaDesired = RadiansPerSecond.of(Math.copySign(0.1, thetaDesired.in(RadiansPerSecond)));
                }

                // scale based solely on distance!
                double dist = Math.hypot(xToGoal, yToGoal);
                double distDesiredPID = altXyPid.calculate(dist);
                Translation2d transToGoal = transformToGoal.getTranslation().times(distDesiredPID);

                // packaging as a Transform2d because we don't have access to gyro here so
                // cannot do ChassisSpeeds
                Transform2d alignmentSpeeds = new Transform2d(xDesired.in(MetersPerSecond),
                                yDesired.in(MetersPerSecond),
                                new Rotation2d(-thetaDesired.in(RadiansPerSecond)));
                return () -> alignmentSpeeds;
        }
}
