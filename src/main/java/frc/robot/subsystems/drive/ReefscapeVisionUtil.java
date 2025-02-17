package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.Degrees;

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
import frc.robot.RobotContainer.AlignType;

public class ReefscapeVisionUtil {
        private static final Pose2d RED_REEF_A = new Pose2d(new Translation2d(),
                        new Rotation2d(Degrees.of(180)));
        private static final Pose2d RED_REEF_B = new Pose2d(new Translation2d(),
                        new Rotation2d(Degrees.of(180)));
        private static final Pose2d RED_REEF_C = new Pose2d(new Translation2d(),
                        new Rotation2d(Degrees.of(240)));
        private static final Pose2d RED_REEF_D = new Pose2d(new Translation2d(),
                        new Rotation2d(Degrees.of(240)));
        private static final Pose2d RED_REEF_E = new Pose2d(new Translation2d(),
                        new Rotation2d(Degrees.of(300)));
        private static final Pose2d RED_REEF_F = new Pose2d(new Translation2d(),
                        new Rotation2d(Degrees.of(300)));
        private static final Pose2d RED_REEF_G = new Pose2d(new Translation2d(11.23, 4.18),
                        new Rotation2d(Degrees.of(180)));
        private static final Pose2d RED_REEF_H = new Pose2d(new Translation2d(11.23, 3.56),
                        new Rotation2d(Degrees.of(180))); // TODO: SWAP THESE BACK TO 0!
        private static final Pose2d RED_REEF_I = new Pose2d(new Translation2d(),
                        new Rotation2d(Degrees.of(60)));
        private static final Pose2d RED_REEF_J = new Pose2d(new Translation2d(),
                        new Rotation2d(Degrees.of(60)));
        private static final Pose2d RED_REEF_K = new Pose2d(new Translation2d(),
                        new Rotation2d(Degrees.of(120)));
        private static final Pose2d RED_REEF_L = new Pose2d(new Translation2d(),
                        new Rotation2d(Degrees.of(120)));

        private static final Pose2d BLUE_REEF_A = new Pose2d(new Translation2d(),
                        new Rotation2d(Degrees.of(0)));
        private static final Pose2d BLUE_REEF_B = new Pose2d(new Translation2d(),
                        new Rotation2d(Degrees.of(0)));
        private static final Pose2d BLUE_REEF_C = new Pose2d(new Translation2d(),
                        new Rotation2d(Degrees.of(60)));
        private static final Pose2d BLUE_REEF_D = new Pose2d(new Translation2d(),
                        new Rotation2d(Degrees.of(60)));
        private static final Pose2d BLUE_REEF_E = new Pose2d(new Translation2d(),
                        new Rotation2d(Degrees.of(120)));
        private static final Pose2d BLUE_REEF_F = new Pose2d(new Translation2d(),
                        new Rotation2d(Degrees.of(120)));
        private static final Pose2d BLUE_REEF_G = new Pose2d(new Translation2d(),
                        new Rotation2d(Degrees.of(180)));
        private static final Pose2d BLUE_REEF_H = new Pose2d(new Translation2d(),
                        new Rotation2d(Degrees.of(180)));
        private static final Pose2d BLUE_REEF_I = new Pose2d(new Translation2d(),
                        new Rotation2d(Degrees.of(240)));
        private static final Pose2d BLUE_REEF_J = new Pose2d(new Translation2d(),
                        new Rotation2d(Degrees.of(240)));
        private static final Pose2d BLUE_REEF_K = new Pose2d(new Translation2d(),
                        new Rotation2d(Degrees.of(300)));
        private static final Pose2d BLUE_REEF_L = new Pose2d(new Translation2d(),
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

        public static ArrayList<Pose2d> getAlignmentPoses() {
                AprilTagFieldLayout tagLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);
                ArrayList<Pose2d> tagList = new ArrayList<Pose2d>();
                tagList.add(tagLayout.getTagPose(7).get().toPose2d());
                tagList.add(tagLayout.getTagPose(8).get().toPose2d());
                tagList.add(tagLayout.getTagPose(9).get().toPose2d());
                tagList.add(tagLayout.getTagPose(10).get().toPose2d());
                tagList.add(tagLayout.getTagPose(11).get().toPose2d());
                tagList.add(tagLayout.getTagPose(6).get().toPose2d());
                tagList.add(tagLayout.getTagPose(18).get().toPose2d());
                tagList.add(tagLayout.getTagPose(17).get().toPose2d());
                tagList.add(tagLayout.getTagPose(22).get().toPose2d());
                tagList.add(tagLayout.getTagPose(21).get().toPose2d());
                tagList.add(tagLayout.getTagPose(20).get().toPose2d());
                tagList.add(tagLayout.getTagPose(19).get().toPose2d());
                ArrayList<Pose2d> backedOff = backedOff(tagList);
                ArrayList<Pose2d> split;

                return tagList;
        }

        public static ArrayList<Pose2d> backedOff(ArrayList<Pose2d> apriltags) {
                double backoffDist = 0.20; // distance off the reef to score in METERS
                ArrayList<Pose2d> backed = new ArrayList<Pose2d>();
                for (Pose2d tag : apriltags) {
                        Pose2d backingOff = tag.plus(new Transform2d( // 0 for rotation bc we're going off the tags pose, which has
                                                         // our correct desired rotation. but for beta, we will have to
                                                         // change them by a factor of 180 degrees
                                        backoffDist * Math.sin(tag.getRotation().getRadians()), // if our theta is 0, we want to move straight back, so x should not change
                                        backoffDist * Math.cos(tag.getRotation().getRadians()),
                                        new Rotation2d(0)));
                        System.out.println(backingOff);
                }
                return backed;
        }
}
