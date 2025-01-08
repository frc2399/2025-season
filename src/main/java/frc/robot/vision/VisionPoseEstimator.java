package frc.robot.vision;

import java.util.Optional;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import frc.robot.Constants.SpeedConstants;

public final class VisionPoseEstimator {
    private static final double CAMERA_PITCH_RADIANS = Units.degreesToRadians(45);
    public static final double X_OFFSET_TO_ROBOT_M = -0.063;
    public static final double Y_OFFSET_TO_ROBOT_M = -0.252;
    public static final double Z_OFFSET_TO_ROBOT_M = 0.278;
    private static final double CAMERA_YAW_RADIANS = Units.degreesToRadians(180);

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

    // meters, radians. Robot origin to camera lens origin
    private static final Transform3d ROBOT_TO_CAMERA = new Transform3d(
            X_OFFSET_TO_ROBOT_M, Y_OFFSET_TO_ROBOT_M, Z_OFFSET_TO_ROBOT_M,
            new Rotation3d(0, CAMERA_PITCH_RADIANS, CAMERA_YAW_RADIANS));

    // reject new poses if spinning too fast
    private static final double MAX_ROTATIONS_PER_SECOND = 2;
    private static final double MAX_VISION_UPDATE_SPEED_MPS = 0.5 * SpeedConstants.DRIVETRAIN_MAX_SPEED_MPS;

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
    public VisionPoseEstimator(DriveBase driveBase, String limelightName) {

        this.driveBase = driveBase;
        this.limelightName = limelightName;
        this.limelightHostname = "limelight" + (limelightName != "" ? "-" + limelightName : "");

        mt2Publisher = NetworkTableInstance.getDefault()
                .getStructTopic("VisionPoseEstimator/" + this.limelightName, Pose2d.struct).publish();
        mt2Publisher.setDefault(new Pose2d());

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
    public VisionPoseEstimator(DriveBase driveBase) {
        this(driveBase, "");
    }

    /**
     * Get a pose estimate from the configured Limelight, if available
     *
     * @return An Optional containing MegaTag2 pose estimate from the Limelight, or
     *         Optional.empty if it is unavailable or untrustworthy
     */
    public Optional<LimelightHelpers.PoseEstimate> getPoseEstimate() {
        if (Math.abs(driveBase.getYawPerSecond().getRotations()) > MAX_ROTATIONS_PER_SECOND) {
            return Optional.empty();
        } else if (driveBase.getLinearSpeed() > MAX_VISION_UPDATE_SPEED_MPS) {
            return Optional.empty();
        }
        var est = Optional.ofNullable(LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightName));
        return est.filter((pe) -> pe.tagCount > 0);
    }

    /**
     * Update the limelight's robot orientation
     */
    public void periodic() {
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