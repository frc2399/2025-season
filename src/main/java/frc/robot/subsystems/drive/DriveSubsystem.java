// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Pose2d;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.SpeedConstants;
import frc.robot.Robot;
import frc.robot.subsystems.gyro.Gyro;
import frc.robot.vision.VisionPoseEstimator.DriveBase;

public class DriveSubsystem extends SubsystemBase implements DriveBase {
        private double velocityXMPS;
        private double velocityYMPS;

        // correction PID
        private double DRIVE_P = 1.1;
        private double DRIVE_D = 0.05;

        private PIDController drivePIDController = new PIDController(DRIVE_P, 0, DRIVE_D);

        // debouncer for turning
        private double ROTATION_DEBOUNCE_TIME = 0.5;
        private Debouncer rotationDebouncer = new Debouncer(ROTATION_DEBOUNCE_TIME);

        private final double MIN_STRAFE_SPEED = 0.075;

        // Odometry
        private SwerveDrivePoseEstimator poseEstimator;

        // swerve modules
        private SwerveModule frontLeft;
        private SwerveModule frontRight;
        private SwerveModule rearLeft;
        private SwerveModule rearRight;

        private final Distance TRACK_WIDTH;
        private final Distance WHEEL_BASE;

        // Distance between front and back wheels on robot

        private final Translation2d FRONT_LEFT_OFFSET;
        private final Translation2d REAR_LEFT_OFFSET;
        private final Translation2d FRONT_RIGHT_OFFSET;
        private final Translation2d REAR_RIGHT_OFFSET;

        private final SwerveDriveKinematics DRIVE_KINEMATICS;

        // Slew rate filter variables for controlling lateral acceleration
        private double currentRotationRate = 0.0;
        private double desiredAngle = 0;
        private Gyro gyro;

        private final Field2d field2d = new Field2d();
        private FieldObject2d frontLeftField2dModule = field2d.getObject("front left module");
        private FieldObject2d rearLeftField2dModule = field2d.getObject("rear left module");
        private FieldObject2d frontRightField2dModule = field2d.getObject("front right module");
        private FieldObject2d rearRightField2dModule = field2d.getObject("rear right module");

        private ChassisSpeeds relativeRobotSpeeds = new ChassisSpeeds();

        private Rotation2d lastAngle = new Rotation2d();

        StructArrayPublisher<SwerveModuleState> swerveModuleStatePublisher = NetworkTableInstance.getDefault()
                        .getStructArrayTopic("/SmartDashboard/Swerve/Current Modules States", SwerveModuleState.struct)
                        .publish();

        StructArrayPublisher<SwerveModuleState> swerveModuleDesiredStatePublisher = NetworkTableInstance.getDefault()
                        .getStructArrayTopic("/SmartDashboard/Swerve/Desired Modules States", SwerveModuleState.struct)
                        .publish();

        /** Creates a new DriveSubsystem. */
        public DriveSubsystem(SwerveModule frontLeft, SwerveModule frontRight, SwerveModule rearLeft,
                        SwerveModule rearRight, Gyro gyro, Distance trackWidth) {
                this.gyro = gyro;
                this.frontLeft = frontLeft;
                this.frontRight = frontRight;
                this.rearLeft = rearLeft;
                this.rearRight = rearRight;

                TRACK_WIDTH = trackWidth;
                WHEEL_BASE = trackWidth;

                FRONT_LEFT_OFFSET = new Translation2d(WHEEL_BASE.in(Meters) / 2,
                        TRACK_WIDTH.in(Meters) / 2);
                REAR_LEFT_OFFSET = new Translation2d(-WHEEL_BASE.in(Meters) / 2,
                        TRACK_WIDTH.in(Meters) / 2);
                FRONT_RIGHT_OFFSET = new Translation2d(WHEEL_BASE.in(Meters) / 2,
                -TRACK_WIDTH.in(Meters) / 2);
                REAR_RIGHT_OFFSET = new Translation2d(-WHEEL_BASE.in(Meters) / 2,
                        -TRACK_WIDTH.in(Meters) / 2);

                DRIVE_KINEMATICS = new SwerveDriveKinematics(
                        FRONT_LEFT_OFFSET,
                        FRONT_RIGHT_OFFSET,
                        REAR_LEFT_OFFSET,
                        REAR_RIGHT_OFFSET);

                SmartDashboard.putData(field2d);

                poseEstimator = new SwerveDrivePoseEstimator(
                                DRIVE_KINEMATICS,
                                Rotation2d.fromDegrees(gyro.getYaw()),
                                new SwerveModulePosition[] {
                                                frontLeft.getPosition(),
                                                frontRight.getPosition(),
                                                rearLeft.getPosition(),
                                                rearRight.getPosition() },
                                new Pose2d(0, 0, new Rotation2d(0, 0))); // TODO: make these constants in the constants
                                                                         // file rather than
                                                                         // free-floating numbers

                configurePathPlannerLogging();
        }

        @Override
        public void periodic() {
                // This will get the simulated sensor readings that we set
                // in the previous article while in simulation, but will use
                // real values on the robot itself.
                SmartDashboard.putNumber("left front distance (meters)", frontLeft.getDriveEncoderPosition());
                SmartDashboard.putNumber("drive/gyro angle(degrees)", Math.toDegrees(gyro.getYaw()));
                poseEstimator.updateWithTime(Timer.getFPGATimestamp(), Rotation2d.fromRadians(gyro.getYaw()),
                                new SwerveModulePosition[] {
                                                frontLeft.getPosition(),
                                                frontRight.getPosition(),
                                                rearLeft.getPosition(),
                                                rearRight.getPosition()
                                });

                Pose2d pose = getPose();
                SmartDashboard.putNumber("Swerve/vision/x", pose.getX());
                SmartDashboard.putNumber("Swerve/vision/y", pose.getY());

                SmartDashboard.putNumber("robot pose theta", pose.getRotation().getDegrees());
                field2d.setRobotPose(pose);

                frontLeftField2dModule.setPose(pose.transformBy(new Transform2d(
                                FRONT_LEFT_OFFSET,
                                new Rotation2d(frontLeft.getTurnEncoderPosition()))));

                rearLeftField2dModule.setPose(pose.transformBy(new Transform2d(
                                REAR_LEFT_OFFSET,
                                new Rotation2d(rearLeft.getTurnEncoderPosition()))));

                frontRightField2dModule.setPose(pose.transformBy(new Transform2d(
                                FRONT_RIGHT_OFFSET,
                                new Rotation2d(frontRight.getTurnEncoderPosition()))));

                rearRightField2dModule.setPose(pose.transformBy(new Transform2d(
                                REAR_RIGHT_OFFSET,
                                new Rotation2d(rearRight.getTurnEncoderPosition()))));

                SwerveModuleState[] swerveModuleStates = new SwerveModuleState[] {
                                frontLeft.getState(),
                                frontRight.getState(),
                                rearLeft.getState(),
                                rearRight.getState(),
                };
                swerveModuleStatePublisher.set(swerveModuleStates);

                if (Robot.isSimulation()) {
                        double angleChange = DRIVE_KINEMATICS
                                        .toChassisSpeeds(swerveModuleStates).omegaRadiansPerSecond
                                        * (1 / Constants.SpeedConstants.MAIN_LOOP_FREQUENCY_HZ);
                        lastAngle = lastAngle.plus(Rotation2d.fromRadians(angleChange));
                        gyro.setYaw(lastAngle.getRadians());
                }
        }

        /** Returns the currently-estimated pose of the robot. */
        public Pose2d getPose() {
                return poseEstimator.getEstimatedPosition();
        }

        /** Returns the current odometry rotation. */
        public Rotation2d getRotation() {
                return getPose().getRotation();

        }

        /** Resets the odometry to the specified pose. */
        public void resetOdometry(Pose2d pose) {
                poseEstimator.resetPosition(
                                Rotation2d.fromRadians(gyro.getYaw()),
                                new SwerveModulePosition[] {
                                                frontLeft.getPosition(),
                                                frontRight.getPosition(),
                                                rearLeft.getPosition(),
                                                rearRight.getPosition()
                                },
                                pose);
        }

        /**
         * Method to drive the robot using joystick info.
         *
         * @param xSpeed        Speed of the robot in the x direction (forward).
         * @param ySpeed        Speed of the robot in the y direction (sideways).
         * @param rotRate       Angular rate of the robot.
         * @param fieldRelative Whether the provided x and y speeds are relative to the
         *                      field.
         */
        public void drive(double xSpeed, double ySpeed, double rotRate, boolean fieldRelative) {
                rotRate = Math.pow(rotRate, 5);
                double newRotRate = 0;
                double currentAngle = (gyro.getYaw());
                double r = Math.pow(Math.sqrt(Math.pow(xSpeed, 2) + Math.pow(ySpeed, 2)), 3);
                double polarAngle = Math.atan2(ySpeed, xSpeed);
                double polarXSpeed = r * Math.cos(polarAngle);
                double polarYSpeed = r * Math.sin(polarAngle);

                // //Account for edge case when gyro resets
                if (currentAngle == 0) {
                        desiredAngle = 0;
                }

                newRotRate = getHeadingCorrectionRotRate(currentAngle, rotRate, polarXSpeed, polarYSpeed);

                // Convert the commanded speeds into the correct units for the drivetrain
                double xSpeedDelivered = polarXSpeed * SpeedConstants.DRIVETRAIN_MAX_SPEED_MPS;
                double ySpeedDelivered = polarYSpeed * SpeedConstants.DRIVETRAIN_MAX_SPEED_MPS;
                double rotRateDelivered = newRotRate * SpeedConstants.DRIVETRAIN_MAX_ANGULAR_SPEED_RPS;

                if (fieldRelative) {
                        relativeRobotSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered,
                                        rotRateDelivered,
                                        Rotation2d.fromRadians(gyro.getYaw()));
                } else {
                        relativeRobotSpeeds = new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotRateDelivered);
                }

                SmartDashboard.putNumber("Swerve/velocity",
                                Math.sqrt(
                                                Math.pow(relativeRobotSpeeds.vxMetersPerSecond, 2)
                                                                + Math.pow(relativeRobotSpeeds.vyMetersPerSecond, 2)));

                var swerveModuleStates = DRIVE_KINEMATICS.toSwerveModuleStates(relativeRobotSpeeds);
                SwerveDriveKinematics.desaturateWheelSpeeds(
                                swerveModuleStates, SpeedConstants.DRIVETRAIN_MAX_SPEED_MPS);
                frontLeft.setDesiredState(swerveModuleStates[0]);
                frontRight.setDesiredState(swerveModuleStates[1]);
                rearLeft.setDesiredState(swerveModuleStates[2]);
                rearRight.setDesiredState(swerveModuleStates[3]);

                swerveModuleDesiredStatePublisher.set(swerveModuleStates);
        }

        /**
         * Sets the wheels into an X formation to prevent movement.
         */
        public Command setX() {
                return this.run(() -> {
                        frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
                        frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
                        rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
                        rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
                });
        }

        public ChassisSpeeds getRobotRelativeSpeeds() {
                return DRIVE_KINEMATICS.toChassisSpeeds(frontLeft.getState(), frontRight.getState(),
                                rearLeft.getState(), rearRight.getState());
        }

        public void setRobotRelativeSpeeds(ChassisSpeeds speeds) {
                speeds = ChassisSpeeds.discretize(speeds, .02);
                var swerveModuleStates = DRIVE_KINEMATICS.toSwerveModuleStates(speeds);
                SwerveDriveKinematics.desaturateWheelSpeeds(
                                swerveModuleStates, SpeedConstants.DRIVETRAIN_MAX_SPEED_MPS);
                frontLeft.setDesiredState(swerveModuleStates[0]);
                frontRight.setDesiredState(swerveModuleStates[1]);
                rearLeft.setDesiredState(swerveModuleStates[2]);
                rearRight.setDesiredState(swerveModuleStates[3]);
                swerveModuleDesiredStatePublisher.set(swerveModuleStates);

        }

        private void configurePathPlannerLogging() {
                PathPlannerLogging.setLogCurrentPoseCallback((pose) -> {
                        field2d.setRobotPose(pose);
                });

                PathPlannerLogging.setLogTargetPoseCallback((pose) -> {
                        field2d.getObject("ROBOT target pose").setPose(pose);
                });

                PathPlannerLogging.setLogActivePathCallback((poses) -> {
                        field2d.getObject("ROBOT path").setPoses(poses);
                });
        }

        private double getHeadingCorrectionRotRate(double currentAngle, double rotRate, double polarXSpeed,
                        double polarYSpeed) {
                // Debouncer ensures that there is no back-correction immediately after turning
                // Deadband for small movements - they are so slight they do not need correction
                // and correction causes robot to spasm
                double newRotRate = 0;
                if (rotationDebouncer.calculate(rotRate == 0)
                                && (Math.abs(polarXSpeed) >= MIN_STRAFE_SPEED
                                                || Math.abs(polarYSpeed) >= MIN_STRAFE_SPEED)) {
                        newRotRate = newRotRate + drivePIDController.calculate(currentAngle, desiredAngle);
                } else {
                        newRotRate = rotRate;
                        desiredAngle = currentAngle;
                }
                return newRotRate;
        }

        @Override
        public Rotation2d getYaw() {
                return new Rotation2d(gyro.getYaw());
        }

        @Override
        public Rotation2d getYawPerSecond() {
                return new Rotation2d(gyro.getAngularVelocity().getValueAsDouble());
        }

        @Override
        public double getLinearSpeed() {
                velocityXMPS = getRobotRelativeSpeeds().vxMetersPerSecond;
                velocityYMPS = getRobotRelativeSpeeds().vyMetersPerSecond;
                return Math.sqrt((Math.pow(velocityXMPS, 2) + Math.pow(velocityYMPS, 2)));
        }

        @Override
        public void addVisionMeasurement(Pose2d pose, double timestampSeconds,
                        Matrix<N3, N1> visionMeasurementStdDevs) {
                poseEstimator.addVisionMeasurement(pose, timestampSeconds, visionMeasurementStdDevs);
        }
}