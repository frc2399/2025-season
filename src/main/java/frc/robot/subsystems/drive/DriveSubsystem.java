// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.Optional;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.hal.SimBoolean;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.CommandFactory.RobotPosition;
import frc.robot.Constants;
import frc.robot.Constants.DriveControlConstants;
import frc.robot.Constants.SpeedConstants;
import frc.robot.Robot;
import frc.robot.subsystems.gyro.Gyro;
import frc.robot.vision.VisionPoseEstimator.DriveBase;

public class DriveSubsystem extends SubsystemBase implements DriveBase {
        // for drivetopose
        private AtomicBoolean atGoal = new AtomicBoolean(true);
        private BooleanSupplier isBlueAlliance;

        private DriveSubsystemStates states = new DriveSubsystemStates();

        // correction PID
        private double DRIVE_P = 1.1;
        private double DRIVE_D = 0.05;

        private PIDController drivePIDController = new PIDController(DRIVE_P, 0, DRIVE_D);

        private RobotConfig config;

        // debouncer for turning
        private double ROTATION_DEBOUNCE_TIME = 0.5;
        private Debouncer rotationDebouncer = new Debouncer(ROTATION_DEBOUNCE_TIME);

        private final double MIN_STRAFE_SPEED = 0.075;

        // Odometry
        private SwerveDrivePoseEstimator poseEstimator;
        private Pose2d robotPose;

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

        private static final double HOLONOMIC_P = 5.0;
        private static final double HOLONOMIC_I = 0.0;
        private static final double HOLONOMIC_D = 0.0;

        // Slew rate filter variables for controlling lateral acceleration
        private double currentRotationRate = 0.0;
        private double desiredAngle = 0;
        private Gyro gyro;

        Alert alert = new Alert("Fault detected on pigeon", AlertType.kError);

        private final Field2d field2d = new Field2d();
        private FieldObject2d frontLeftField2dModule = field2d.getObject("front left module");
        private FieldObject2d rearLeftField2dModule = field2d.getObject("rear left module");
        private FieldObject2d frontRightField2dModule = field2d.getObject("front right module");
        private FieldObject2d rearRightField2dModule = field2d.getObject("rear right module");

        private ChassisSpeeds relativeRobotSpeeds = new ChassisSpeeds();

        private Rotation2d lastAngle = new Rotation2d();

        public static class DriveSubsystemStates {
                public Pose2d pose = new Pose2d();
                public double poseTheta = 0;
                public double velocityXMPS = 0;
                public double velocityYMPS = 0;
                public double totalVelocity = 0;
                public double gyroAngleDegrees = 0;
                public double angularVelocity = 0;
                public double driveEncoderPos = 0.0;
        }

        StructArrayPublisher<SwerveModuleState> swerveModuleStatePublisher = NetworkTableInstance.getDefault()
                        .getStructArrayTopic("/SmartDashboard/Swerve/Current Modules States", SwerveModuleState.struct)
                        .publish();

        StructArrayPublisher<SwerveModuleState> swerveModuleDesiredStatePublisher = NetworkTableInstance.getDefault()
                        .getStructArrayTopic("/SmartDashboard/Swerve/Desired Modules States", SwerveModuleState.struct)
                        .publish();

        // Useful pose debugging
        private final StructPublisher<Pose2d> posePublisher;

        /** Creates a new DriveSubsystem. */
        public DriveSubsystem(SwerveModule frontLeft, SwerveModule frontRight, SwerveModule rearLeft,
                        SwerveModule rearRight, Gyro gyro, Distance trackWidth, Distance wheelBase) {
                this.gyro = gyro;
                this.frontLeft = frontLeft;
                this.frontRight = frontRight;
                this.rearLeft = rearLeft;
                this.rearRight = rearRight;

                TRACK_WIDTH = trackWidth;
                WHEEL_BASE = wheelBase;

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
                                Rotation2d.fromDegrees(gyro.getYaw(false).in(Degrees)),
                                new SwerveModulePosition[] {
                                                frontLeft.getPosition(),
                                                frontRight.getPosition(),
                                                rearLeft.getPosition(),
                                                rearRight.getPosition() },
                                new Pose2d(0, 0, new Rotation2d(gyro.getYaw()))); // TODO: make these constants in the
                                                                                  // constants
                // file rather than
                // free-floating numbers
                posePublisher = NetworkTableInstance.getDefault()
                                .getStructTopic("DriveSubsystem/EstimatedPose", Pose2d.struct).publish();
                posePublisher.setDefault(new Pose2d());
                try {
                        config = RobotConfig.fromGUISettings();

                        AutoBuilder.configure(
                                        this::getPose,
                                        this::resetOdometry,
                                        this::getRobotRelativeSpeeds,
                                        (speeds, feedforwards) -> setRobotRelativeSpeeds(speeds),
                                        new PPHolonomicDriveController(
                                                        new PIDConstants(HOLONOMIC_P, HOLONOMIC_I,
                                                                        HOLONOMIC_D),
                                                        new PIDConstants(HOLONOMIC_P, HOLONOMIC_I,
                                                                        HOLONOMIC_D)),
                                        config, // The robot configuration
                                        () -> {

                                                var alliance = DriverStation.getAlliance();
                                                if (alliance.isPresent()) {
                                                        return alliance.get() == DriverStation.Alliance.Red;
                                                }
                                                return false;
                                        },
                                        this);
                } catch (Exception e) {
                        DriverStation.reportError("Failed to load Pathplanner config and configure Autobuilder",
                                        e.getStackTrace());
                }
                configurePathPlannerLogging();
        }

        @Override
        public void periodic() {
                SmartDashboard.putBoolean("/drive/atGoal", atGoal.get());
                // This will get the simulated sensor readings that we set
                // in the previous article while in simulation, but will use
                // real values on the robot itself.
                poseEstimator.updateWithTime(Timer.getFPGATimestamp(),
                                Rotation2d.fromRadians(gyro.getYaw(true).in(Radians)),
                                new SwerveModulePosition[] {
                                                frontLeft.getPosition(),
                                                frontRight.getPosition(),
                                                rearLeft.getPosition(),
                                                rearRight.getPosition()
                                });

                robotPose = getPose();
                field2d.setRobotPose(robotPose);
                logAndUpdateDriveSubsystemStates();

                frontLeftField2dModule.setPose(robotPose.transformBy(new Transform2d(
                                FRONT_LEFT_OFFSET,
                                new Rotation2d(frontLeft.getTurnEncoderPosition()))));

                rearLeftField2dModule.setPose(robotPose.transformBy(new Transform2d(
                                REAR_LEFT_OFFSET,
                                new Rotation2d(rearLeft.getTurnEncoderPosition()))));

                frontRightField2dModule.setPose(robotPose.transformBy(new Transform2d(
                                FRONT_RIGHT_OFFSET,
                                new Rotation2d(frontRight.getTurnEncoderPosition()))));

                rearRightField2dModule.setPose(robotPose.transformBy(new Transform2d(
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
                        gyro.setYaw(Radians.of(lastAngle.getRadians()));
                }

                logAndUpdateDriveSubsystemStates();
                alert.set(gyro.hasFault());

                frontLeft.updateStates();
                frontRight.updateStates();
                rearLeft.updateStates();
                rearRight.updateStates();
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
                                Rotation2d.fromRadians(gyro.getYaw(false).in(Radians)),
                                new SwerveModulePosition[] {
                                                frontLeft.getPosition(),
                                                frontRight.getPosition(),
                                                rearLeft.getPosition(),
                                                rearRight.getPosition()
                                },
                                pose);
        }

        // basically, when we reset gyro, m_gyroOffset in SwerveDrivePoseEstimator does
        // NOT reset so the pose does not reset properly :(
        public void resetOdometryAfterGyro() {
                poseEstimator.resetRotation(Rotation2d.fromRadians(gyro.getYaw(false).in(Radians)));
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
        public Command driveCommand(DoubleSupplier xSpeed, DoubleSupplier ySpeed, DoubleSupplier rotRate,
                        Boolean fieldRelative, BooleanSupplier isElevatorHigh) {
                return this.run(() -> {
                        Boolean useFieldRelative = true;
                        double driveSpeedFactor = DriveControlConstants.DRIVE_FACTOR;
                        if (isElevatorHigh.getAsBoolean()) {
                                driveSpeedFactor = DriveControlConstants.SLOW_DRIVE_FACTOR;
                                // useFieldRelative = false;
                        }
                        double currentAngle = gyro.getYaw(false).in(Radians);
                        if (DriverStation.getAlliance().isPresent()
                                        && DriverStation.getAlliance().get() == Alliance.Red) {
                                currentAngle += Math.PI;
                        }

                        double r = Math.hypot(xSpeed.getAsDouble() * driveSpeedFactor,
                                        ySpeed.getAsDouble() * driveSpeedFactor);
                        double polarAngle = Math.atan2(ySpeed.getAsDouble(), xSpeed.getAsDouble());
                        double polarXSpeed = r * Math.cos(polarAngle);
                        double polarYSpeed = r * Math.sin(polarAngle);

                        // //Account for edge case when gyro resets
                        if (currentAngle == 0) {
                                desiredAngle = 0;
                        }

                        double newRotRate = getHeadingCorrectionRotRate(currentAngle,
                                        Math.pow(rotRate.getAsDouble(), 5),
                                        polarXSpeed, polarYSpeed);

                        // Convert the commanded speeds into the correct units for the drivetrain
                        double xSpeedDelivered = polarXSpeed * SpeedConstants.DRIVETRAIN_MAX_SPEED_MPS;
                        double ySpeedDelivered = polarYSpeed * SpeedConstants.DRIVETRAIN_MAX_SPEED_MPS;
                        double rotRateDelivered = newRotRate * SpeedConstants.DRIVETRAIN_MAX_ANGULAR_SPEED_RPS;

                        if (useFieldRelative) {
                                relativeRobotSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered,
                                                ySpeedDelivered,
                                                rotRateDelivered,
                                                Rotation2d.fromRadians(currentAngle));
                        } else {
                                relativeRobotSpeeds = new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered,
                                                rotRateDelivered);
                        }

                        SmartDashboard.putNumber("x speed delivered", xSpeedDelivered);
                        SmartDashboard.putNumber("y speed delivered", ySpeedDelivered);
                        SmartDashboard.putNumber("drive speed factor", driveSpeedFactor);

                        var swerveModuleStates = DRIVE_KINEMATICS.toSwerveModuleStates(relativeRobotSpeeds);
                        SwerveDriveKinematics.desaturateWheelSpeeds(
                                        swerveModuleStates, SpeedConstants.DRIVETRAIN_MAX_SPEED_MPS);
                        frontLeft.setDesiredState(swerveModuleStates[0]);
                        frontRight.setDesiredState(swerveModuleStates[1]);
                        rearLeft.setDesiredState(swerveModuleStates[2]);
                        rearRight.setDesiredState(swerveModuleStates[3]);

                        swerveModuleDesiredStatePublisher.set(swerveModuleStates);
                }).withName("drive command");
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
                return new Rotation2d(gyro.getYaw(false));
        }

        @Override
        public Rotation2d getYawPerSecond() {
                return new Rotation2d(gyro.getAngularVelocity().getValueAsDouble());
        }

        @Override
        public double getLinearSpeed() {
                double velocityXMPS = getRobotRelativeSpeeds().vxMetersPerSecond;
                double velocityYMPS = getRobotRelativeSpeeds().vyMetersPerSecond;
                return Math.hypot(velocityXMPS, velocityYMPS);
        }

        @Override
        public void addVisionMeasurement(Pose2d pose, double timestampSeconds,
                        Matrix<N3, N1> visionMeasurementStdDevs) {
                // Something cursed happens here where the robot code crashes in a loop on first
                // boot, complaining about doing a Rotation2d.exp on a Rotation2d with x = y =
                // 0. I (Will) _think_ it has to do with an invalid timestamp value passed here
                // (negative? before robot boot?) that then causes the poseEstimator to try to
                // replay odometry measurements that it doesn't have. This try-catch fixes the
                // issue, and who wants vision updates to crash their robot code anyway?

                if (timestampSeconds <= 0) {
                        return;
                }
                poseEstimator.addVisionMeasurement(pose, timestampSeconds, visionMeasurementStdDevs);
        }

        // this method has a LOT of suppliers - so short explanation of why they're good
        // basically, when a button binding is called in robotContainer, it makes an
        // object
        // represnting the command. this object does not change, and it does not
        // actually look
        // at the command each time - it just calls the one that was constructed at
        // robotInit
        // by using a supplier, the robot knows 'hey, this value might change, so i
        // should
        // check it every time i use this object' thus allowing it to change
        public Command driveToPoseCommand(Supplier<RobotPosition> robotPosition) {
                return this.run(() -> {
                        // basically, bad things can happen if we try to update a normal boolean within
                        // a lambda and access it outside that lambda, but atomic booleans prevent these
                        // risks
                        atGoal.set(false);

                        if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Blue) {
                                isBlueAlliance = () -> true;
                        } else {
                                isBlueAlliance = () -> false;
                        }

                        Supplier<Pose2d> goalPose = ReefscapeVisionUtil.getGoalPose(robotPosition.get(),
                                        () -> robotPose,
                                        isBlueAlliance);
                        SmartDashboard.putNumber("Swerve/vision/goalPoseY", goalPose.get().getY());
                        SmartDashboard.putNumber("Swerve/vision/goalPosex", goalPose.get().getX());
                        SmartDashboard.putNumber("Swerve/vision/goalTheta", goalPose.get().getRotation().getDegrees());

                        Supplier<Transform2d> velocities = DriveToPoseUtil.getDriveToPoseVelocities(
                                        () -> robotPose, goalPose);
                        ChassisSpeeds alignmentSpeeds = new ChassisSpeeds(
                                        velocities.get().getX(),
                                        velocities.get().getY(),
                                        velocities.get().getRotation().getRadians());

                        SmartDashboard.putNumber("Swerve/vision/xVel", alignmentSpeeds.vxMetersPerSecond);
                        SmartDashboard.putNumber("Swerve/vision/yVel", alignmentSpeeds.vyMetersPerSecond);
                        SmartDashboard.putNumber("Swerve/vision/thetaVel", alignmentSpeeds.omegaRadiansPerSecond);

                        // tolerances were accounted for in getDriveToPoseVelocities
                        atGoal.set((velocities.get().getX() == 0 && velocities.get().getY() == 0
                                        && velocities.get().getRotation().getRadians() == 0));

                        setRobotRelativeSpeeds(alignmentSpeeds);
                }).until(() -> atGoal.get());
        }

        public Command disableDriveToPose() {
                return this.runOnce(() -> {
                        atGoal.set(true);
                        frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(0)));
                        frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(0)));
                        rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(0)));
                        rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(0)));
                });
        }

        private void logAndUpdateDriveSubsystemStates() {
                states.pose = getPose();
                states.poseTheta = states.pose.getRotation().getDegrees();
                states.velocityXMPS = getRobotRelativeSpeeds().vxMetersPerSecond;
                states.velocityYMPS = getRobotRelativeSpeeds().vyMetersPerSecond;
                states.totalVelocity = Math.hypot(states.velocityXMPS, states.velocityYMPS);
                states.angularVelocity = Units.radiansToDegrees(relativeRobotSpeeds.omegaRadiansPerSecond);
                states.gyroAngleDegrees = gyro.getYaw(false).in(Degrees);

                // Publish the pose in a struct that can be laid onto the "odometry" view in
                // advantagescope
                posePublisher.set(states.pose);

                SmartDashboard.putNumber("drive/Pose X(m)", states.pose.getX());
                SmartDashboard.putNumber("drive/Pose Y(m)", states.pose.getY());
                SmartDashboard.putNumber("drive/Pose Theta(deg)", states.poseTheta);
                SmartDashboard.putNumber("drive/Linear Velocity X(mps)", states.velocityXMPS);
                SmartDashboard.putNumber("drive/Linear Velocity Y(mps)", states.velocityYMPS);
                SmartDashboard.putNumber("drive/Total Velocity(mps)", states.totalVelocity);
                SmartDashboard.putNumber("drive/Angular Velocity(deg per sec)", states.angularVelocity);
                SmartDashboard.putNumber("drive/Gyro Angle(deg)", states.gyroAngleDegrees);
        }
}
