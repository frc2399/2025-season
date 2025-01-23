package frc.robot.subsystems.drive;

import java.util.Arrays;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.RobotContainer.AlignType;

public class ReefscapeVisionUtil {
    private static final Pose2d RED_REEF_A = new Pose2d();
    private static final Pose2d RED_REEF_B = new Pose2d();
    private static final Pose2d RED_REEF_C = new Pose2d();
    private static final Pose2d RED_REEF_D = new Pose2d();
    private static final Pose2d RED_REEF_E = new Pose2d();
    private static final Pose2d RED_REEF_F = new Pose2d();
    private static final Pose2d RED_REEF_G = new Pose2d();
    private static final Pose2d RED_REEF_H = new Pose2d();
    private static final Pose2d RED_REEF_I = new Pose2d();
    private static final Pose2d RED_REEF_J = new Pose2d();
    private static final Pose2d RED_REEF_K = new Pose2d();
    private static final Pose2d RED_REEF_L = new Pose2d();

    private static final Pose2d BLUE_REEF_A = new Pose2d();
    private static final Pose2d BLUE_REEF_B = new Pose2d();
    private static final Pose2d BLUE_REEF_C = new Pose2d();
    private static final Pose2d BLUE_REEF_D = new Pose2d();
    private static final Pose2d BLUE_REEF_E = new Pose2d();
    private static final Pose2d BLUE_REEF_F = new Pose2d();
    private static final Pose2d BLUE_REEF_G = new Pose2d();
    private static final Pose2d BLUE_REEF_H = new Pose2d();
    private static final Pose2d BLUE_REEF_I = new Pose2d();
    private static final Pose2d BLUE_REEF_J = new Pose2d();
    private static final Pose2d BLUE_REEF_K = new Pose2d();
    private static final Pose2d BLUE_REEF_L = new Pose2d();

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

    public static Pose2d getGoalPose(AlignType alignType, Pose2d robotPose, boolean isBlueAlliance) {
        Pose2d goalPose;
        if (isBlueAlliance) {
            if (alignType == AlignType.CORAL_STATION) {
                goalPose = BLUE_CORAL_STATION;
            } else if (alignType == AlignType.REEF_LEFT) {
                goalPose = robotPose.nearest(LEFT_POSES_BLUE);
            } else {
                goalPose = robotPose.nearest(RIGHT_POSES_BLUE);
            }
        } else {
            if (alignType == AlignType.CORAL_STATION) {
                goalPose = RED_CORAL_STATION;
            } else if (alignType == AlignType.REEF_LEFT) {
                goalPose = robotPose.nearest(LEFT_POSES_RED);
            } else {
                goalPose = robotPose.nearest(RIGHT_POSES_RED);
            }
        }
        return goalPose;
    }
}
