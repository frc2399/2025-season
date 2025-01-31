package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Rotation2d;

public interface SwerveModuleIO {

    public static class SwerveModuleIOStates {

        public double driveVoltage = 0.0;
        public double turnVoltage = 0.0;
        public double drivingVelocity = 0.0;
        public double desiredDrivingVelocity = 0.0;
        public double turningPosition = 0.0;
        public double desiredAngle = 0.0;
        public double speed = 0.0;
        public double driveCurrent = 0.0;
        public double turnCurrent = 0.0;
        public Rotation2d yawPerSec = new Rotation2d();
        public double drivingEncoderPos = 0.0;
        public double turningEncoderPos = 0.0;

        
        

    }

    public void setDriveEncoderPosition(double position);


    public void setDesiredDriveSpeedMPS(double speed);

    public double getDriveEncoderSpeedMPS();
    
    public double getTurnEncoderSpeedMPS();

    public double getTurnCurrent();
    public void setDesiredTurnAngle(double angle);

    public double getDriveOutput();
    public double getTurnEncoderPosition();
    public double getDriveEncoderPosition();

    public double getTurnOutput();


    public void updateStates(SwerveModuleIOStates states) ;

    public double getChassisAngularOffset();

}