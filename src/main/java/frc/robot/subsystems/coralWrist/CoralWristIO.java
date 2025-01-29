package frc.robot.subsystems.coralWrist;

import edu.wpi.first.units.measure.Angle;

public interface CoralWristIO {
    static class CoralWristIOStates {
        public double wristVelocity = 0.0;
        public double wristAppliedVoltage = 0.0;
        public double wristCurrent = 0.0;
        public double wristAbsoluteEncoderAngle = 0.0;
    }

    public void goToSetpoint(Angle angle);

    public void setWristSpeed(double speed);

    public void updateStates(CoralWristIOStates states);
}
