package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.function.Supplier;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;
import frc.robot.RobotContainer.AlignType;

public class ReefscapeVisionUtil {
        private static final Pose2d RED_REEF_A = new Pose2d(new Translation2d(14.538, 3.861),
                        new Rotation2d(Degrees.of(0)));
        private static final Pose2d RED_REEF_B = new Pose2d(new Translation2d(14.538, 4.191),
                        new Rotation2d(Degrees.of(0)));
        private static final Pose2d RED_REEF_C = new Pose2d(new Translation2d(13.942, 5.225),
                        new Rotation2d(Degrees.of(60)));
        private static final Pose2d RED_REEF_D = new Pose2d(new Translation2d(13.656, 5.390),
                        new Rotation2d(Degrees.of(60)));
        private static final Pose2d RED_REEF_E = new Pose2d(new Translation2d(12.462, 5.390),
                        new Rotation2d(Degrees.of(120)));
        private static final Pose2d RED_REEF_F = new Pose2d(new Translation2d(12.176, 5.225),
                        new Rotation2d(Degrees.of(120)));
        private static final Pose2d RED_REEF_G = new Pose2d(new Translation2d(11.579, 4.191),
                        new Rotation2d(Degrees.of(180)));
        private static final Pose2d RED_REEF_H = new Pose2d(new Translation2d(11.579, 3.861),
                        new Rotation2d(Degrees.of(180))); // TODO: SWAP THESE BACK TO 0!
        private static final Pose2d RED_REEF_I = new Pose2d(new Translation2d(12.176, 2.827),
                        new Rotation2d(Degrees.of(-120)));
        private static final Pose2d RED_REEF_J = new Pose2d(new Translation2d(12.462, 2.662),
                        new Rotation2d(Degrees.of(-120)));
        private static final Pose2d RED_REEF_K = new Pose2d(new Translation2d(13.656, 2.662),
                        new Rotation2d(Degrees.of(-60)));
        private static final Pose2d RED_REEF_L = new Pose2d(new Translation2d(13.942, 2.827),
                        new Rotation2d(Degrees.of(-60)));

        private static final Pose2d BLUE_REEF_A = new Pose2d(new Translation2d(3.010, 4.191),
                        new Rotation2d(Degrees.of(0)));
        private static final Pose2d BLUE_REEF_B = new Pose2d(new Translation2d(3.010, 3.861),
                        new Rotation2d(Degrees.of(0)));
        private static final Pose2d BLUE_REEF_C = new Pose2d(new Translation2d(3.607, 2.827),
                        new Rotation2d(Degrees.of(60)));
        private static final Pose2d BLUE_REEF_D = new Pose2d(new Translation2d(3.893, 2.662),
                        new Rotation2d(Degrees.of(60)));
        private static final Pose2d BLUE_REEF_E = new Pose2d(new Translation2d(5.086, 2.662),
                        new Rotation2d(Degrees.of(120)));
        private static final Pose2d BLUE_REEF_F = new Pose2d(new Translation2d(5.372, 2.827),
                        new Rotation2d(Degrees.of(120)));
        private static final Pose2d BLUE_REEF_G = new Pose2d(new Translation2d(5.969, 3.861),
                        new Rotation2d(Degrees.of(180)));
        private static final Pose2d BLUE_REEF_H = new Pose2d(new Translation2d(5.969, 4.191),
                        new Rotation2d(Degrees.of(180)));
        private static final Pose2d BLUE_REEF_I = new Pose2d(new Translation2d(5.372, 5.225),
                        new Rotation2d(Degrees.of(240)));
        private static final Pose2d BLUE_REEF_J = new Pose2d(new Translation2d(5.086, 5.390),
                        new Rotation2d(Degrees.of(240)));
        private static final Pose2d BLUE_REEF_K = new Pose2d(new Translation2d(3.893, 5.390),
                        new Rotation2d(Degrees.of(300)));
        private static final Pose2d BLUE_REEF_L = new Pose2d(new Translation2d(3.607, 5.225),
                        new Rotation2d(Degrees.of(300)));

        private static final Pose2d RED_CORAL_STATION = new Pose2d();
        private static final Pose2d BLUE_CORAL_STATION = new Pose2d();

        private static final List<Pose2d> LEFT_POSES_RED = Arrays.asList(
                        RED_REEF_A, RED_REEF_C, RED_REEF_E, RED_REEF_G, RED_REEF_I, RED_REEF_K);
        private static final List<Pose2d> RIGHT_POSES_RED = Arrays.asList(
                        RED_REEF_B, RED_REEF_D, RED_REEF_F, RED_REEF_H, RED_REEF_J, RED_REEF_L);

        private static final List<Pose2d> LEFT_POSES_BLUE = Arrays.asList(
                        BLUE_REEF_A, BLUE_REEF_C, BLUE_REEF_E, BLUE_REEF_G, BLUE_REEF_I, BLUE_REEF_K);

        private static final List<Pose2d> RIGHT_POSES_BLUE = Arrays.asList(
                        BLUE_REEF_B, BLUE_REEF_D, BLUE_REEF_F, BLUE_REEF_H, BLUE_REEF_J, BLUE_REEF_L);

        public static Supplier<Pose2d> getGoalPose(AlignType alignType, Supplier<Pose2d> robotPose,
                        boolean isBlueAlliance) {
                Pose2d goalPose;
                if (robotPose.get() == null) {
                        Pose2d nullReturn = new Pose2d();
                        return () -> nullReturn;
                }
                if (isBlueAlliance) {
                        if (alignType == AlignType.CORAL_STATION) {
                                goalPose = BLUE_CORAL_STATION;
                        } else if (alignType == AlignType.REEF_LEFT) {
                                goalPose = robotPose.get().nearest(LEFT_POSES_BLUE);
                        } else {
                                goalPose = robotPose.get().nearest(RIGHT_POSES_BLUE);
                        }
                } else {
                        if (alignType == AlignType.CORAL_STATION) {
                                goalPose = RED_CORAL_STATION;
                        } else if (alignType == AlignType.REEF_LEFT) {
                                goalPose = robotPose.get().nearest(LEFT_POSES_RED);
                        } else {
                                goalPose = robotPose.get().nearest(RIGHT_POSES_RED);
                        }
                }
                Supplier<Pose2d> goalPoseSupplier = () -> goalPose;
                return goalPoseSupplier;
        }
}
