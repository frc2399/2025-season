package frc.robot.vision;

import static edu.wpi.first.units.Units.Degree;
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
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SpeedConstants;

public final class VisionPoseEstimator extends SubsystemBase {
    // mozart values
    // private static final Angle CAMERA_PITCH =
    // Degrees.of(24.62);
    // private static final Distance X_ROBOT_TO_CAMERA_OFFSET =
    // Inches.of(-11.94);
    // this is positive instead of negative despite robot coordinate system due to
    // the 180* yaw rotation!
    // private static final Distance Y_ROBOT_TO_CAMERA_OFFSET =
    // Inches.of(7.54);
    // private static final Distance Z_ROBOT_TO_CAMERA_OFFSET =
    // Inches.of(4.937);
    // private static final Angle CAMERA_YAW = Degrees.of(180);
 
    //TODO: change these when we get actual values for a robot!
    private static final Angle CAMERA_PITCH = Degrees.of(0);
    private static final Distance X_ROBOT_TO_CAMERA_OFFSET = Inches.of(0);
    private static final Distance Y_ROBOT_TO_CAMERA_OFFSET = Inches.of(0);
    private static final Distance Z_ROBOT_TO_CAMERA_OFFSET = Inches.of(0);
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

    // meters, radians. Robot origin to camera lens origin
    private static final Transform3d ROBOT_TO_CAMERA = new Transform3d(
            X_ROBOT_TO_CAMERA_OFFSET.in(Meters), Y_ROBOT_TO_CAMERA_OFFSET.in(Meters),
            Z_ROBOT_TO_CAMERA_OFFSET.in(Meters),
            new Rotation3d(0, CAMERA_PITCH.in(Radians), CAMERA_YAW.in(Radians)));

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
        if (Math.abs(driveBase.getYawPerSecond().getRotations()) > MAX_ROTATIONS_PER_SECOND.in(RotationsPerSecond)) {
            return Optional.empty();
        } else if (driveBase.getLinearSpeed() > MAX_DRIVETRAIN_SPEED_FOR_VISION_UPDATE.in(MetersPerSecond)) {
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