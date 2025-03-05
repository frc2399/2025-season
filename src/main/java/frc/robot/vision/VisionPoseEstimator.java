package frc.robot.vision;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.Optional;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SpeedConstants;
import frc.robot.SubsystemFactory.RobotType;

public final class VisionPoseEstimator extends SubsystemBase {

    private static final Angle CAMERA_PITCH = Degrees.of(28);
    private static final Distance X_ROBOT_TO_CAMERA_OFFSET = Inches.of(11.175);
    private static Distance Y_ROBOT_TO_CAMERA_OFFSET;
    private static final Distance Z_ROBOT_TO_CAMERA_OFFSET = Inches.of(6.405);
    private static final Angle CAMERA_YAW = Degrees.of(0);

    /**
     * Provides the methods needed to do first-class pose estimation
     */
    public static interface DriveBase {

        Rotation2d getYaw();

        Rotation2d getYawPerSecond();

        double getLinearSpeed();

        /**
         * Passthrough to {@link edu.wpi.first.math.estimator.SwerveDrivePoseEstimator}
         * addVisionMeasurement
         *
         * @param pose
         * @param timestampSeconds
         * @param visionMeasurementStdDevs
         */
        void addVisionMeasurement(Pose2d pose, double timestampSeconds, Matrix<N3, N1> visionMeasurementStdDevs);
    }

    private static Transform3d ROBOT_TO_CAMERA;

    // reject new poses if spinning too fast
    private static final AngularVelocity MAX_ROTATIONS_PER_SECOND = RotationsPerSecond.of(2);
    private static final LinearVelocity MAX_DRIVETRAIN_SPEED_FOR_VISION_UPDATE = MetersPerSecond
            .of(0.5 * SpeedConstants.DRIVETRAIN_MAX_SPEED_MPS);

    private final StructPublisher<Pose2d> mt2Publisher;
    private final DriveBase driveBase;
    private final String limelightName, limelightHostname;

    /**
     * Create a VisionPoseEstimator
     *
     * @param driveBase     the robot drive base to estimate the pose of
     * @param limelightName passed down to calls to LimelightHelpers, useful if you
     *                      have more than one Limelight on a robot
     */
    public VisionPoseEstimator(DriveBase driveBase, String limelightName, RobotType robot) {
        this.driveBase = driveBase;
        this.limelightName = limelightName;
        this.limelightHostname = "limelight" + (limelightName != "" ? "-" + limelightName : "");

        mt2Publisher = NetworkTableInstance.getDefault()
                .getStructTopic("VisionPoseEstimator/" + this.limelightName, Pose2d.struct).publish();
        mt2Publisher.setDefault(new Pose2d());

        if (robot == RobotType.BETA) {
            Y_ROBOT_TO_CAMERA_OFFSET = Inches.of(-2);
        } else {
            Y_ROBOT_TO_CAMERA_OFFSET = Inches.of(0);
        }

        // meters, radians. Robot origin to camera lens origin
        ROBOT_TO_CAMERA = new Transform3d(
                X_ROBOT_TO_CAMERA_OFFSET.in(Meters), Y_ROBOT_TO_CAMERA_OFFSET.in(Meters),
                Z_ROBOT_TO_CAMERA_OFFSET.in(Meters),
                new Rotation3d(0, CAMERA_PITCH.in(Radians), CAMERA_YAW.in(Radians)));

        LimelightHelpers.setCameraPose_RobotSpace(limelightName, ROBOT_TO_CAMERA.getX(), ROBOT_TO_CAMERA.getY(),
                ROBOT_TO_CAMERA.getZ(), Math.toDegrees(ROBOT_TO_CAMERA.getRotation().getX()),
                Math.toDegrees(ROBOT_TO_CAMERA.getRotation().getY()),
                Math.toDegrees(ROBOT_TO_CAMERA.getRotation().getZ()));
    }

    /**
     * Create a VisionPoseEstimator
     *
     * @param driveBase the robot drive base to estimate the pose of
     */
    public VisionPoseEstimator(DriveBase driveBase, RobotType robotType) {
        this(driveBase, "", robotType);
    }

    /**
     * Get a pose estimate from the configured Limelight, if available
     *
     * @return An Optional containing MegaTag2 pose estimate from the Limelight, or
     *         Optional.empty if it is unavailable or untrustworthy
     */
    public Optional<LimelightHelpers.PoseEstimate> getPoseEstimate() {
        if (Math.abs(driveBase.getYawPerSecond().getRotations()) > MAX_ROTATIONS_PER_SECOND.in(RotationsPerSecond)) {
            return Optional.empty();
        } else if (driveBase.getLinearSpeed() > MAX_DRIVETRAIN_SPEED_FOR_VISION_UPDATE.in(MetersPerSecond)) {
            return Optional.empty();
        }
        var est = Optional.ofNullable(LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightName));
        // Reject poses where we can see no tags or are at the "uh oh something went wrong" 0,0 coordinate
        return est.filter((pe) -> pe.tagCount > 0 && (pe.pose.getX() != 0 && pe.pose.getY() != 0));
    }

    /**
     * Update the limelight's robot orientation
     */
    public void periodic() {
        // Resist the temptation to rotate this depending on alliance - the coordinate
        // system here _has_ to match the WPILib coordinate system, where Yaw is CCW +
        // and 0 faces the red alliance wall
        LimelightHelpers.SetRobotOrientation(limelightName, driveBase.getYaw().getDegrees(), 0, 0, 0, 0, 0);
        getPoseEstimate().ifPresent((pe) -> {
            mt2Publisher.set(pe.pose);
            // LimelightHelpers doesn't expose a helper method for these, layout is:
            // [MT1x, MT1y, MT1z, MT1roll, MT1pitch, MT1Yaw, MT2x, MT2y, MT2z, MT2roll,
            // MT2pitch, MT2yaw]
            var stddevs = LimelightHelpers.getLimelightNTDoubleArray(limelightHostname, "stddevs");
            driveBase.addVisionMeasurement(pe.pose, pe.timestampSeconds,
                    VecBuilder.fill(stddevs[6], stddevs[7], Double.POSITIVE_INFINITY));
        });
    }
}