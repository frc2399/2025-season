package frc.robot.subsystems.coralIntake;

import edu.wpi.first.units.measure.AngularVelocity;

public interface CoralIntakeIO {
    static class CoralIntakeIOStates {
        public double velocity = 0.0;
        public double leftCurrent = 0.0;
        public double rightCurrent = 0.0;
        public double leftAppliedVoltage = 0.0;
        public double rightAppliedVoltage = 0.0;
    }

    public void setRollerSpeed(AngularVelocity speed);

    public void setSpeedType(AngularVelocity speed);

    public void updateStates(CoralIntakeIOStates states);
}