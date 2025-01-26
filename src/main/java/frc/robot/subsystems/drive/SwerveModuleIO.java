package frc.robot.subsystems.drive;

public interface SwerveModuleIO {
   
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

    public double getChassisAngularOffset();

    public String getName();
}