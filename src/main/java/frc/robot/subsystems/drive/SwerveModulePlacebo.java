package frc.robot.subsystems.drive;

public class SwerveModulePlacebo implements SwerveModuleIO {

    public void setDriveEncoderPosition(double position) {}

    public double getDriveEncoderPosition() {
        return 0.0;
    }

    public void setDesiredDriveSpeedMPS(double speed) {}

    public double getDriveEncoderSpeedMPS() {
        return 0.0;
    }

    public double getTurnEncoderPosition() {
        return 0.0;
    }

    public void setDesiredTurnAngle(double angle) {}

    public double getDriveBusVoltage() {
        return 0.0;
    }

    public double getDriveOutput() {
        return 0.0;
    }

    public double getTurnBusVoltage() {
        return 0.0;
    }

    public double getTurnOutput() {
        return 0.0;
    }

    public double getChassisAngularOffset() {
        return 0.0;
    }

    public String getName() {
        return "";
    }

    public void updateStates(){
        
    }

    public double getDriveCurrent(){
        return 0.0;

    }

    public double getTurnCurrent(){
        return 0.0;
    }

    
    public void updateStates(SwerveModuleIOStates states) {
        
    }
}