package frc.robot.subsystems.algaeWrist;

import java.util.function.Supplier;

import frc.robot.CommandFactory.Setpoint;

public interface AlgaeWristIO {

    static class AlgaeWristIOStates {
        public double wristVelocity = 0.0;
        public double wristCurrent = 0.0;
        public double wristAppliedVoltage = 0.0;
        public double wristAbsoluteEncoderAngle = 0.0;
        public double wristRelativeEncoderAngle = 0.0;
        public double goalAngle = 0.0;
    }

    public void setGoalAngle(Supplier<Setpoint> setpoint);

    public void setWristSpeed(double speed);

    public void updateStates(AlgaeWristIOStates states);

    public void periodic();
}
