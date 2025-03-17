package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.Inches;

import java.util.Arrays;
import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import frc.robot.CommandFactory.RobotPosition;

public class ReefscapeVisionUtil {
        private static final Pose2d TAG_9 = new Pose2d(12.643, 4.475, Rotation2d.fromDegrees(240));
        private static final Pose2d TAG_10 = new Pose2d(12.227, 4.026, Rotation2d.k180deg);
        private static final Pose2d TAG_11 = new Pose2d(12.643, 3.306, Rotation2d.fromDegrees(-240));

        private static final Transform2d REEF_TO_ROBOT = new Transform2d(Inches.of(31.5), Inches.zero(),
                        Rotation2d.k180deg);
        private static final Transform2d SCORING_POSE_OFFSET = new Transform2d(Inches.zero(), Inches.of(7.5),
                        Rotation2d.kZero);

        private static final Pose2d RED_REEF_E = TAG_9.transformBy(REEF_TO_ROBOT.plus(SCORING_POSE_OFFSET));
        private static final Pose2d RED_REEF_F = TAG_9.transformBy(REEF_TO_ROBOT.plus(SCORING_POSE_OFFSET.inverse()));
        private static final Pose2d RED_REEF_G = TAG_10.transformBy(REEF_TO_ROBOT.plus(SCORING_POSE_OFFSET));
        private static final Pose2d RED_REEF_H = TAG_10.transformBy(REEF_TO_ROBOT.plus(SCORING_POSE_OFFSET.inverse()));
        private static final Pose2d RED_REEF_I = TAG_11.transformBy(REEF_TO_ROBOT.plus(SCORING_POSE_OFFSET));
        private static final Pose2d RED_REEF_J = TAG_11.transformBy(REEF_TO_ROBOT.plus(SCORING_POSE_OFFSET.inverse()));
        // private static final List<Pose2d> LEFT_POSES_RED = Arrays.asList(
        // RED_REEF_A, RED_REEF_C, RED_REEF_E, RED_REEF_G, RED_REEF_I, RED_REEF_K);
        // private static final List<Pose2d> RIGHT_POSES_RED = Arrays.asList(
        // RED_REEF_B, RED_REEF_D, RED_REEF_F, RED_REEF_H, RED_REEF_J, RED_REEF_L);
        private static final List<Pose2d> LEFT_POSES_RED = Arrays.asList(
                        RED_REEF_E, RED_REEF_G, RED_REEF_I);
        private static final List<Pose2d> RIGHT_POSES_RED = Arrays.asList(
                        RED_REEF_F, RED_REEF_H, RED_REEF_J);

        // private static final List<Pose2d> LEFT_POSES_BLUE = Arrays.asList(
        // BLUE_REEF_A, BLUE_REEF_C, BLUE_REEF_E, BLUE_REEF_G, BLUE_REEF_I,
        // BLUE_REEF_K);

        // private static final List<Pose2d> RIGHT_POSES_BLUE = Arrays.asList(
        // BLUE_REEF_B, BLUE_REEF_D, BLUE_REEF_F, BLUE_REEF_H, BLUE_REEF_J,
        // BLUE_REEF_L);

        public static Supplier<Pose2d> getGoalPose(RobotPosition robotPosition, Supplier<Pose2d> robotPose,
                        BooleanSupplier isBlueAlliance) {
                Pose2d goalPose;
                if (robotPose.get() == null) {
                        Pose2d nullReturn = new Pose2d();
                        return () -> nullReturn;
                }
                if (isBlueAlliance.getAsBoolean()) {
                        if (robotPosition == RobotPosition.LEFT) {
                                // goalPose = robotPose.get().nearest(LEFT_POSES_BLUE);
                                goalPose = new Pose2d(0, 0, new Rotation2d(0));
                        } else {
                                // goalPose = robotPose.get().nearest(RIGHT_POSES_BLUE);
                                goalPose = new Pose2d(0, 0, new Rotation2d(0));
                        }
                } else {
                        if (robotPosition == RobotPosition.LEFT) {
                                goalPose = robotPose.get().nearest(LEFT_POSES_RED);
                        } else {
                                goalPose = robotPose.get().nearest(RIGHT_POSES_RED);
                        }
                }
                Supplier<Pose2d> goalPoseSupplier = () -> goalPose;
                return goalPoseSupplier;
        }
}
