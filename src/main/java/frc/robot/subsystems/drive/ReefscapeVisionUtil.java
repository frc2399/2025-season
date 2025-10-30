package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.Inches;

import java.util.Arrays;
import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import frc.robot.CommandFactory.AutomatedScoringPoseLocation;
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

        private static final Transform2d FAR_REEF_TO_ROBOT = new Transform2d(Inches.of(29.5), Inches.zero(),
                        Rotation2d.k180deg);
        private static final Transform2d NEAR_REEF_TO_ROBOT = new Transform2d(Inches.of(25), Inches.zero(),
                        Rotation2d.k180deg);

        private static final Transform2d SCORING_POSE_OFFSET_LEFT = new Transform2d(Inches.zero(), Inches.of(6.5),
                        Rotation2d.kZero);
        private static final Transform2d SCORING_POSE_OFFSET_RIGHT = new Transform2d(Inches.zero(), Inches.of(-6.5),
                        Rotation2d.kZero);

        private static final Pose2d RED_REEF_A_NEAR = TAG_7.transformBy(NEAR_REEF_TO_ROBOT.plus(SCORING_POSE_OFFSET_LEFT));
        private static final Pose2d RED_REEF_B_NEAR = TAG_7.transformBy(NEAR_REEF_TO_ROBOT.plus(SCORING_POSE_OFFSET_RIGHT));
        private static final Pose2d RED_REEF_C_NEAR = TAG_8.transformBy(NEAR_REEF_TO_ROBOT.plus(SCORING_POSE_OFFSET_LEFT));
        private static final Pose2d RED_REEF_D_NEAR = TAG_8.transformBy(NEAR_REEF_TO_ROBOT.plus(SCORING_POSE_OFFSET_RIGHT));
        private static final Pose2d RED_REEF_E_NEAR = TAG_9.transformBy(NEAR_REEF_TO_ROBOT.plus(SCORING_POSE_OFFSET_LEFT));
        private static final Pose2d RED_REEF_F_NEAR = TAG_9.transformBy(NEAR_REEF_TO_ROBOT.plus(SCORING_POSE_OFFSET_RIGHT));
        private static final Pose2d RED_REEF_G_NEAR = TAG_10.transformBy(NEAR_REEF_TO_ROBOT.plus(SCORING_POSE_OFFSET_LEFT));
        private static final Pose2d RED_REEF_H_NEAR = TAG_10.transformBy(NEAR_REEF_TO_ROBOT.plus(SCORING_POSE_OFFSET_RIGHT));
        private static final Pose2d RED_REEF_I_NEAR = TAG_11.transformBy(NEAR_REEF_TO_ROBOT.plus(SCORING_POSE_OFFSET_LEFT));
        private static final Pose2d RED_REEF_J_NEAR = TAG_11.transformBy(NEAR_REEF_TO_ROBOT.plus(SCORING_POSE_OFFSET_RIGHT));
        private static final Pose2d RED_REEF_K_NEAR = TAG_6.transformBy(NEAR_REEF_TO_ROBOT.plus(SCORING_POSE_OFFSET_LEFT));
        private static final Pose2d RED_REEF_L_NEAR = TAG_6.transformBy(NEAR_REEF_TO_ROBOT.plus(SCORING_POSE_OFFSET_RIGHT));

        private static final Pose2d BLUE_REEF_A_NEAR = TAG_18.transformBy(NEAR_REEF_TO_ROBOT.plus(SCORING_POSE_OFFSET_LEFT));
        private static final Pose2d BLUE_REEF_B_NEAR = TAG_18.transformBy(NEAR_REEF_TO_ROBOT.plus(SCORING_POSE_OFFSET_RIGHT));
        private static final Pose2d BLUE_REEF_C_NEAR = TAG_19.transformBy(NEAR_REEF_TO_ROBOT.plus(SCORING_POSE_OFFSET_LEFT));
        private static final Pose2d BLUE_REEF_D_NEAR = TAG_19.transformBy(NEAR_REEF_TO_ROBOT.plus(SCORING_POSE_OFFSET_RIGHT));
        private static final Pose2d BLUE_REEF_E_NEAR = TAG_20.transformBy(NEAR_REEF_TO_ROBOT.plus(SCORING_POSE_OFFSET_LEFT));
        private static final Pose2d BLUE_REEF_F_NEAR = TAG_20.transformBy(NEAR_REEF_TO_ROBOT.plus(SCORING_POSE_OFFSET_RIGHT));
        private static final Pose2d BLUE_REEF_G_NEAR = TAG_21.transformBy(NEAR_REEF_TO_ROBOT.plus(SCORING_POSE_OFFSET_LEFT));
        private static final Pose2d BLUE_REEF_H_NEAR = TAG_21.transformBy(NEAR_REEF_TO_ROBOT.plus(SCORING_POSE_OFFSET_RIGHT));
        private static final Pose2d BLUE_REEF_I_NEAR = TAG_22.transformBy(NEAR_REEF_TO_ROBOT.plus(SCORING_POSE_OFFSET_LEFT));
        private static final Pose2d BLUE_REEF_J_NEAR = TAG_22.transformBy(NEAR_REEF_TO_ROBOT.plus(SCORING_POSE_OFFSET_RIGHT));
        private static final Pose2d BLUE_REEF_K_NEAR = TAG_17.transformBy(NEAR_REEF_TO_ROBOT.plus(SCORING_POSE_OFFSET_LEFT));
        private static final Pose2d BLUE_REEF_L_NEAR = TAG_17.transformBy(NEAR_REEF_TO_ROBOT.plus(SCORING_POSE_OFFSET_RIGHT));

        private static final Pose2d RED_REEF_A_FAR = TAG_7.transformBy(FAR_REEF_TO_ROBOT.plus(SCORING_POSE_OFFSET_LEFT));
        private static final Pose2d RED_REEF_B_FAR = TAG_7.transformBy(FAR_REEF_TO_ROBOT.plus(SCORING_POSE_OFFSET_RIGHT));
        private static final Pose2d RED_REEF_C_FAR = TAG_8.transformBy(FAR_REEF_TO_ROBOT.plus(SCORING_POSE_OFFSET_LEFT));
        private static final Pose2d RED_REEF_D_FAR = TAG_8.transformBy(FAR_REEF_TO_ROBOT.plus(SCORING_POSE_OFFSET_RIGHT));
        private static final Pose2d RED_REEF_E_FAR = TAG_9.transformBy(FAR_REEF_TO_ROBOT.plus(SCORING_POSE_OFFSET_LEFT));
        private static final Pose2d RED_REEF_F_FAR = TAG_9.transformBy(FAR_REEF_TO_ROBOT.plus(SCORING_POSE_OFFSET_RIGHT));
        private static final Pose2d RED_REEF_G_FAR = TAG_10.transformBy(FAR_REEF_TO_ROBOT.plus(SCORING_POSE_OFFSET_LEFT));
        private static final Pose2d RED_REEF_H_FAR = TAG_10.transformBy(FAR_REEF_TO_ROBOT.plus(SCORING_POSE_OFFSET_RIGHT));
        private static final Pose2d RED_REEF_I_FAR = TAG_11.transformBy(FAR_REEF_TO_ROBOT.plus(SCORING_POSE_OFFSET_LEFT));
        private static final Pose2d RED_REEF_J_FAR = TAG_11.transformBy(FAR_REEF_TO_ROBOT.plus(SCORING_POSE_OFFSET_RIGHT));
        private static final Pose2d RED_REEF_K_FAR = TAG_6.transformBy(FAR_REEF_TO_ROBOT.plus(SCORING_POSE_OFFSET_LEFT));
        private static final Pose2d RED_REEF_L_FAR = TAG_6.transformBy(FAR_REEF_TO_ROBOT.plus(SCORING_POSE_OFFSET_RIGHT));

        private static final Pose2d BLUE_REEF_A_FAR = TAG_18.transformBy(FAR_REEF_TO_ROBOT.plus(SCORING_POSE_OFFSET_LEFT));
        private static final Pose2d BLUE_REEF_B_FAR = TAG_18.transformBy(FAR_REEF_TO_ROBOT.plus(SCORING_POSE_OFFSET_RIGHT));
        private static final Pose2d BLUE_REEF_C_FAR = TAG_19.transformBy(FAR_REEF_TO_ROBOT.plus(SCORING_POSE_OFFSET_LEFT));
        private static final Pose2d BLUE_REEF_D_FAR = TAG_19.transformBy(FAR_REEF_TO_ROBOT.plus(SCORING_POSE_OFFSET_RIGHT));
        private static final Pose2d BLUE_REEF_E_FAR = TAG_20.transformBy(FAR_REEF_TO_ROBOT.plus(SCORING_POSE_OFFSET_LEFT));
        private static final Pose2d BLUE_REEF_F_FAR = TAG_20.transformBy(FAR_REEF_TO_ROBOT.plus(SCORING_POSE_OFFSET_RIGHT));
        private static final Pose2d BLUE_REEF_G_FAR = TAG_21.transformBy(FAR_REEF_TO_ROBOT.plus(SCORING_POSE_OFFSET_LEFT));
        private static final Pose2d BLUE_REEF_H_FAR = TAG_21.transformBy(FAR_REEF_TO_ROBOT.plus(SCORING_POSE_OFFSET_RIGHT));
        private static final Pose2d BLUE_REEF_I_FAR = TAG_22.transformBy(FAR_REEF_TO_ROBOT.plus(SCORING_POSE_OFFSET_LEFT));
        private static final Pose2d BLUE_REEF_J_FAR = TAG_22.transformBy(FAR_REEF_TO_ROBOT.plus(SCORING_POSE_OFFSET_RIGHT));
        private static final Pose2d BLUE_REEF_K_FAR = TAG_17.transformBy(FAR_REEF_TO_ROBOT.plus(SCORING_POSE_OFFSET_LEFT));
        private static final Pose2d BLUE_REEF_L_FAR = TAG_17.transformBy(FAR_REEF_TO_ROBOT.plus(SCORING_POSE_OFFSET_RIGHT));

        private static final List<Pose2d> LEFT_POSES_RED_NEAR = Arrays.asList(
                        RED_REEF_A_NEAR, RED_REEF_C_NEAR, RED_REEF_E_NEAR, RED_REEF_G_NEAR, RED_REEF_I_NEAR, RED_REEF_K_NEAR);

        private static final List<Pose2d> RIGHT_POSES_RED_NEAR = Arrays.asList(
                        RED_REEF_B_NEAR, RED_REEF_D_NEAR, RED_REEF_F_NEAR, RED_REEF_H_NEAR, RED_REEF_J_NEAR, RED_REEF_L_NEAR);

        private static final List<Pose2d> LEFT_POSES_RED_FAR = Arrays.asList(
                  RED_REEF_A_FAR, RED_REEF_C_FAR, RED_REEF_E_FAR, RED_REEF_G_FAR, RED_REEF_I_FAR, RED_REEF_K_FAR);
        
        private static final List<Pose2d> RIGHT_POSES_RED_FAR = Arrays.asList(
                        RED_REEF_B_FAR, RED_REEF_D_FAR, RED_REEF_F_FAR, RED_REEF_H_FAR, RED_REEF_J_FAR, RED_REEF_L_FAR);

        private static final List<Pose2d> LEFT_POSES_BLUE_NEAR = Arrays.asList(
                        BLUE_REEF_A_NEAR, BLUE_REEF_C_NEAR, BLUE_REEF_E_NEAR, BLUE_REEF_G_NEAR, BLUE_REEF_I_NEAR, BLUE_REEF_K_NEAR);

        private static final List<Pose2d> RIGHT_POSES_BLUE_NEAR = Arrays.asList(
                        BLUE_REEF_B_NEAR, BLUE_REEF_D_NEAR, BLUE_REEF_F_NEAR, BLUE_REEF_H_NEAR, BLUE_REEF_J_NEAR, BLUE_REEF_L_NEAR);

        private static final List<Pose2d> LEFT_POSES_BLUE_FAR = Arrays.asList(
                        BLUE_REEF_A_FAR, BLUE_REEF_C_FAR, BLUE_REEF_E_FAR, BLUE_REEF_G_FAR, BLUE_REEF_I_FAR, BLUE_REEF_K_FAR);

        private static final List<Pose2d> RIGHT_POSES_BLUE_FAR = Arrays.asList(
                        BLUE_REEF_B_FAR, BLUE_REEF_D_FAR, BLUE_REEF_F_FAR, BLUE_REEF_H_FAR, BLUE_REEF_J_FAR, BLUE_REEF_L_FAR);

        public static Supplier<Pose2d> getGoalPoseFar(RobotPosition robotPosition, Supplier<Pose2d> robotPose,
                        BooleanSupplier isBlueAlliance) {
                Pose2d goalPose;
                if (robotPose.get() == null) {
                        Pose2d nullReturn = new Pose2d();
                        return () -> nullReturn;
                }
                if (isBlueAlliance.getAsBoolean()) {
                        if (robotPosition == RobotPosition.LEFT) {
                                goalPose = robotPose.get().nearest(LEFT_POSES_BLUE_FAR);
                        } else {
                                goalPose = robotPose.get().nearest(RIGHT_POSES_BLUE_FAR);
                        }
                } else {
                        if (robotPosition == RobotPosition.LEFT) {
                                goalPose = robotPose.get().nearest(LEFT_POSES_RED_FAR);
                        } else {
                                goalPose = robotPose.get().nearest(RIGHT_POSES_RED_FAR);
                        }
                }
                Supplier<Pose2d> goalPoseSupplier = () -> goalPose;
                return goalPoseSupplier;
        }

        public static Supplier<Pose2d> getGoalPoseNear(RobotPosition robotPosition, Supplier<Pose2d> robotPose,
                        BooleanSupplier isBlueAlliance) {
                Pose2d goalPose;
                if (robotPose.get() == null) {
                        Pose2d nullReturn = new Pose2d();
                        return () -> nullReturn;
                }
                if (isBlueAlliance.getAsBoolean()) {
                        if (robotPosition == RobotPosition.LEFT) {
                                goalPose = robotPose.get().nearest(LEFT_POSES_BLUE_NEAR);
                        } else {
                                goalPose = robotPose.get().nearest(RIGHT_POSES_BLUE_NEAR);
                        }
                } else {
                        if (robotPosition == RobotPosition.LEFT) {
                                goalPose = robotPose.get().nearest(LEFT_POSES_RED_NEAR);
                        } else {
                                goalPose = robotPose.get().nearest(RIGHT_POSES_RED_NEAR);
                        }
                }
                Supplier<Pose2d> goalPoseSupplier = () -> goalPose;
                return goalPoseSupplier;
        }
}
