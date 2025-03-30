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
        private static final Pose2d TAG_6 = new Pose2d(13.474, 3.306, Rotation2d.fromDegrees(-60));
        private static final Pose2d TAG_7 = new Pose2d(13.89, 4.026, Rotation2d.kZero);
        private static final Pose2d TAG_8 = new Pose2d(13.474, 4.745, Rotation2d.fromDegrees(60));
        private static final Pose2d TAG_9 = new Pose2d(12.643, 4.745, Rotation2d.fromDegrees(-240));
        private static final Pose2d TAG_10 = new Pose2d(12.227, 4.026, Rotation2d.k180deg);
        private static final Pose2d TAG_11 = new Pose2d(12.643, 3.306, Rotation2d.fromDegrees(240));

        private static final Pose2d TAG_17 = new Pose2d(4.074, 3.306, Rotation2d.fromDegrees(-120));
        private static final Pose2d TAG_18 = new Pose2d(3.658, 4.026, Rotation2d.k180deg);
        private static final Pose2d TAG_19 = new Pose2d(4.074, 4.745, Rotation2d.fromDegrees(120));
        private static final Pose2d TAG_20 = new Pose2d(4.905, 4.745, Rotation2d.fromDegrees(60));
        private static final Pose2d TAG_21 = new Pose2d(5.321, 4.026, Rotation2d.kZero);
        private static final Pose2d TAG_22 = new Pose2d(4.905, 3.306, Rotation2d.fromDegrees(-60));

        private static final Transform2d REEF_TO_ROBOT_PRESCORE = new Transform2d(Inches.of(29.5), Inches.zero(),
                        Rotation2d.k180deg);
        private static final Transform2d SCORING_POSE_OFFSET_LEFT = new Transform2d(Inches.zero(), Inches.of(7.5),
                        Rotation2d.kZero);
        private static final Transform2d SCORING_POSE_OFFSET_RIGHT = new Transform2d(Inches.zero(), Inches.of(-5.5),
                        Rotation2d.kZero);

        private static final Pose2d RED_REEF_A_PRESCORE = TAG_7
                        .transformBy(REEF_TO_ROBOT_PRESCORE.plus(SCORING_POSE_OFFSET_LEFT));
        private static final Pose2d RED_REEF_B_PRESCORE = TAG_7
                        .transformBy(REEF_TO_ROBOT_PRESCORE.plus(SCORING_POSE_OFFSET_RIGHT));
        private static final Pose2d RED_REEF_C_PRESCORE = TAG_8
                        .transformBy(REEF_TO_ROBOT_PRESCORE.plus(SCORING_POSE_OFFSET_LEFT));
        private static final Pose2d RED_REEF_D_PRESCORE = TAG_8
                        .transformBy(REEF_TO_ROBOT_PRESCORE.plus(SCORING_POSE_OFFSET_RIGHT));
        private static final Pose2d RED_REEF_E_PRESCORE = TAG_9
                        .transformBy(REEF_TO_ROBOT_PRESCORE.plus(SCORING_POSE_OFFSET_LEFT));
        private static final Pose2d RED_REEF_F_PRESCORE = TAG_9
                        .transformBy(REEF_TO_ROBOT_PRESCORE.plus(SCORING_POSE_OFFSET_RIGHT));
        private static final Pose2d RED_REEF_G_PRESCORE = TAG_10
                        .transformBy(REEF_TO_ROBOT_PRESCORE.plus(SCORING_POSE_OFFSET_LEFT));
        private static final Pose2d RED_REEF_H_PRESCORE = TAG_10
                        .transformBy(REEF_TO_ROBOT_PRESCORE.plus(SCORING_POSE_OFFSET_RIGHT));
        private static final Pose2d RED_REEF_I_PRESCORE = TAG_11
                        .transformBy(REEF_TO_ROBOT_PRESCORE.plus(SCORING_POSE_OFFSET_LEFT));
        private static final Pose2d RED_REEF_J_PRESCORE = TAG_11
                        .transformBy(REEF_TO_ROBOT_PRESCORE.plus(SCORING_POSE_OFFSET_RIGHT));
        private static final Pose2d RED_REEF_K_PRESCORE = TAG_6
                        .transformBy(REEF_TO_ROBOT_PRESCORE.plus(SCORING_POSE_OFFSET_LEFT));
        private static final Pose2d RED_REEF_L_PRESCORE = TAG_6
                        .transformBy(REEF_TO_ROBOT_PRESCORE.plus(SCORING_POSE_OFFSET_RIGHT));

        private static final Pose2d BLUE_REEF_A_PRESCORE = TAG_18
                        .transformBy(REEF_TO_ROBOT_PRESCORE.plus(SCORING_POSE_OFFSET_LEFT));
        private static final Pose2d BLUE_REEF_B_PRESCORE = TAG_18
                        .transformBy(REEF_TO_ROBOT_PRESCORE.plus(SCORING_POSE_OFFSET_RIGHT));
        private static final Pose2d BLUE_REEF_C_PRESCORE = TAG_19
                        .transformBy(REEF_TO_ROBOT_PRESCORE.plus(SCORING_POSE_OFFSET_LEFT));
        private static final Pose2d BLUE_REEF_D_PRESCORE = TAG_19
                        .transformBy(REEF_TO_ROBOT_PRESCORE.plus(SCORING_POSE_OFFSET_RIGHT));
        private static final Pose2d BLUE_REEF_E_PRESCORE = TAG_20
                        .transformBy(REEF_TO_ROBOT_PRESCORE.plus(SCORING_POSE_OFFSET_LEFT));
        private static final Pose2d BLUE_REEF_F_PRESCORE = TAG_20
                        .transformBy(REEF_TO_ROBOT_PRESCORE.plus(SCORING_POSE_OFFSET_RIGHT));
        private static final Pose2d BLUE_REEF_G_PRESCORE = TAG_21
                        .transformBy(REEF_TO_ROBOT_PRESCORE.plus(SCORING_POSE_OFFSET_LEFT));
        private static final Pose2d BLUE_REEF_H_PRESCORE = TAG_21
                        .transformBy(REEF_TO_ROBOT_PRESCORE.plus(SCORING_POSE_OFFSET_RIGHT));
        private static final Pose2d BLUE_REEF_I_PRESCORE = TAG_22
                        .transformBy(REEF_TO_ROBOT_PRESCORE.plus(SCORING_POSE_OFFSET_LEFT));
        private static final Pose2d BLUE_REEF_J_PRESCORE = TAG_22
                        .transformBy(REEF_TO_ROBOT_PRESCORE.plus(SCORING_POSE_OFFSET_RIGHT));
        private static final Pose2d BLUE_REEF_K_PRESCORE = TAG_17
                        .transformBy(REEF_TO_ROBOT_PRESCORE.plus(SCORING_POSE_OFFSET_LEFT));
        private static final Pose2d BLUE_REEF_L_PRESCORE = TAG_17
                        .transformBy(REEF_TO_ROBOT_PRESCORE.plus(SCORING_POSE_OFFSET_RIGHT));

        private static final List<Pose2d> LEFT_POSES_RED_PRESCORE = Arrays.asList(
                        RED_REEF_A_PRESCORE, RED_REEF_C_PRESCORE, RED_REEF_E_PRESCORE, RED_REEF_G_PRESCORE,
                        RED_REEF_I_PRESCORE, RED_REEF_K_PRESCORE);

        private static final List<Pose2d> RIGHT_POSES_RED_PRESCORE = Arrays.asList(
                        RED_REEF_B_PRESCORE, RED_REEF_D_PRESCORE, RED_REEF_F_PRESCORE, RED_REEF_H_PRESCORE,
                        RED_REEF_J_PRESCORE, RED_REEF_L_PRESCORE);

        private static final List<Pose2d> LEFT_POSES_BLUE_PRESCORE = Arrays.asList(
                        BLUE_REEF_A_PRESCORE, BLUE_REEF_C_PRESCORE, BLUE_REEF_E_PRESCORE, BLUE_REEF_G_PRESCORE,
                        BLUE_REEF_I_PRESCORE, BLUE_REEF_K_PRESCORE);

        private static final List<Pose2d> RIGHT_POSES_BLUE_PRESCORE = Arrays.asList(
                        BLUE_REEF_B_PRESCORE, BLUE_REEF_D_PRESCORE, BLUE_REEF_F_PRESCORE, BLUE_REEF_H_PRESCORE,
                        BLUE_REEF_J_PRESCORE, BLUE_REEF_L_PRESCORE);

        public static Supplier<Pose2d> getGoalPose(RobotPosition robotPosition, Supplier<Pose2d> robotPose,
                        BooleanSupplier isBlueAlliance, BooleanSupplier isPrescore) {
                Pose2d goalPose;
                if (robotPose.get() == null) {
                        Pose2d nullReturn = new Pose2d();
                        return () -> nullReturn;
                }
                if (isPrescore.getAsBoolean()) {
                        if (isBlueAlliance.getAsBoolean()) {
                                if (robotPosition == RobotPosition.LEFT) {
                                        goalPose = robotPose.get().nearest(LEFT_POSES_BLUE_PRESCORE);
                                } else {
                                        goalPose = robotPose.get().nearest(RIGHT_POSES_BLUE_PRESCORE);
                                }
                        } else {
                                if (robotPosition == RobotPosition.LEFT) {
                                        goalPose = robotPose.get().nearest(LEFT_POSES_RED_PRESCORE);
                                } else {
                                        goalPose = robotPose.get().nearest(RIGHT_POSES_RED_PRESCORE);
                                }
                        }
                } else {
                        goalPose = new Pose2d();
                }
                Supplier<Pose2d> goalPoseSupplier = () -> goalPose;
                return goalPoseSupplier;
        }
}
