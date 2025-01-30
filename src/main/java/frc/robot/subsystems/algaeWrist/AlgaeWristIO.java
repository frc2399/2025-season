package frc.robot.subsystems.algaeWrist;

public interface AlgaeWristIO {

    static class AlgaeWristIOStates {
        public double wristVelocity = 0.0;
        public double wristCurrent = 0.0;
        public double wristAppliedVoltage = 0.0;
    }

    public void setSpeed(double speed);

    public double getVelocity();

    public double getCurrent();

    public void updateStates(AlgaeWristIOStates states);
}
