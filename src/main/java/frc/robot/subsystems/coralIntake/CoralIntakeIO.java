package frc.robot.subsystems.coralIntake;

public interface CoralIntakeIO {
    static class CoralIntakeIOStates {
        public double velocity = 0.0;
        public double topCurrent = 0.0;
        public double bottomCurrent = 0.0;
        public double wristCurrent = 0.0;
        public double topAppliedVoltage = 0.0;
        public double bottomAppliedVoltage = 0.0;
        public double wristAppliedVoltage = 0.0;
        public double wristEncoderAngle = 0.0;
    }

    public void setRollerSpeed(double speed);

    public double getVelocity();

    public double getCurrent();

    public void updateStates(CoralIntakeIOStates states);
}