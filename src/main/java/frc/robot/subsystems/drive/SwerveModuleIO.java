package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public interface SwerveModuleIO {

    public static class SwerveModuleIOStates {

        public double driveVoltage = 0.0;
        public double turnVoltage = 0.0;
        public double driveVelocity = 0.0;
        public double driveDesiredVelocity = 0.0;
        public double turnAngle = 0.0;
        public double desiredAngle = 0.0;
        public double driveCurrent = 0.0;
        public double turnCurrent = 0.0;
        public double driveEncoderPos = 0.0;
        public double turningEncoderPos = 0.0;

    }

    public static class DriveSubsystemStates {
        public ChassisSpeeds relativeRobotSpeeds = new ChassisSpeeds();
        public Pose2d pose = new Pose2d();
        public double poseY = 0;
        public double poseX = 0;
        public double poseTheta = 0;
        public double velocityXMPS = 0;
        public double velocityYMPS = 0;
        public double totalVelocity = 0;
        public double gyroAngleDegrees = 0;
        public double angularVelocity = 0;
        public double driveEncoderPos = 0.0;
    }

    public void setDriveEncoderPosition(double position);

    public void setDesiredDriveSpeedMPS(double speed);

    public double getDriveEncoderSpeedMPS();

    public void setDesiredTurnAngle(double angle);

    public double getTurnEncoderPosition();

    public double getDriveEncoderPosition();

    public void updateStates(SwerveModuleIOStates states);

    public double getChassisAngularOffset();

}