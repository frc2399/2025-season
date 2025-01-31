package frc.robot.subsystems.algaeWrist;

import edu.wpi.first.units.measure.Angle;

public interface AlgaeWristIO {

    static class AlgaeWristIOStates {
        public double wristVelocity = 0.0;
        public double wristCurrent = 0.0;
        public double wristAppliedVoltage = 0.0;
        public double wristAbsoluteEncoderAngle = 0.0;
    }

    public void setWristSpeed(double speed);

    public void updateStates(AlgaeWristIOStates states);

    public void goToSetpoint(Angle angle);
}
