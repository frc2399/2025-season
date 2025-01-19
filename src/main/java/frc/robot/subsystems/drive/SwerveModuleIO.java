package frc.robot.subsystems.drive;

public interface SwerveModuleIO {
   
    public static class SwerveModuleIOStates{

        public double driveVoltage = 0.0;
        public double turnVoltage = 0.0;
        public double drivingVelocity = 0.0;
        public double desiredDrivingVelocity = 0.0;
        public double turningPosition = 0.0;
        public double desiredAngle = 0.0;
        public double speed = 0.0;
        public double driveCurrent = 0.0;
        public double turnCurrent = 0.0;

        


    }


    public void setDriveEncoderPosition(double position);

    public double getDriveEncoderPosition();

    public void setDesiredDriveSpeedMPS(double speed);

    public double getDriveEncoderSpeedMPS();

    public double getTurnEncoderPosition();

    public void setDesiredTurnAngle(double angle);

    public double getDriveBusVoltage();

    public double getDriveOutput();

    public double getTurnBusVoltage();

    public double getTurnOutput();
    public double getDriveCurrent();
    public double getTurnCurrent();

    public double getChassisAngularOffset();

    




}