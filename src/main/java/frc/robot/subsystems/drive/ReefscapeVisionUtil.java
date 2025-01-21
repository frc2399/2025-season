package frc.robot.subsystems.drive;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class ReefscapeVisionUtil {
    public static final Pose2d REF_REEF_A = new Pose2d();
    public static final Pose2d REF_REEF_B = new Pose2d();
    public static final Pose2d REF_REEF_C = new Pose2d();
    public static final Pose2d REF_REEF_D = new Pose2d();
    public static final Pose2d REF_REEF_E = new Pose2d();
    public static final Pose2d REF_REEF_F = new Pose2d();
    public static final Pose2d REF_REEF_G = new Pose2d();
    public static final Pose2d REF_REEF_H = new Pose2d();
    public static final Pose2d REF_REEF_I = new Pose2d();
    public static final Pose2d REF_REEF_J = new Pose2d();
    public static final Pose2d REF_REEF_K = new Pose2d();
    public static final Pose2d REF_REEF_L = new Pose2d();

    public static final Pose2d BLUE_REEF_A = new Pose2d();
    public static final Pose2d BLUE_REEF_B = new Pose2d();
    public static final Pose2d BLUE_REEF_C = new Pose2d();
    public static final Pose2d BLUE_REEF_D = new Pose2d();
    public static final Pose2d BLUE_REEF_E = new Pose2d();
    public static final Pose2d BLUE_REEF_F = new Pose2d();
    public static final Pose2d BLUE_REEF_G = new Pose2d();
    public static final Pose2d BLUE_REEF_H = new Pose2d();
    public static final Pose2d BLUE_REEF_I = new Pose2d();
    public static final Pose2d BLUE_REEF_J = new Pose2d();
    public static final Pose2d BLUE_REEF_K = new Pose2d();
    public static final Pose2d BLUE_REEF_L = new Pose2d();

    public static final Pose2d RED_CORAL_STATION = new Pose2d();
    public static final Pose2d BLUE_CORAL_STATION = new Pose2d(); 

    public class RelevantPoseByAlliance {
        List<Pose2d> leftPoses;
        List<Pose2d> rightPoses;
        Pose2d coralPose;
        public RelevantPoseByAlliance(Alliance ally) {
            if (ally == Alliance.Blue) {
                leftPoses.add(BLUE_REEF_A);
                leftPoses.add(BLUE_REEF_C);
                leftPoses.add(BLUE_REEF_E);
                leftPoses.add(BLUE_REEF_G);
                leftPoses.add(BLUE_REEF_I);
                leftPoses.add(BLUE_REEF_K);

                rightPoses.add(BLUE_REEF_B);
                rightPoses.add(BLUE_REEF_D);
                rightPoses.add(BLUE_REEF_F);
                rightPoses.add(BLUE_REEF_H);
                rightPoses.add(BLUE_REEF_J);
                rightPoses.add(BLUE_REEF_L);

                coralPose = BLUE_CORAL_STATION;
            }
        }
    }

}
