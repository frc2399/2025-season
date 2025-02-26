package frc.robot.subsystems.coralIntake;

public interface CoralIntakeIO {
    static class CoralIntakeIOStates {
        public double velocity = 0.0;
        public double goalVelocity = 0.0;
        public double leftCurrent = 0.0;
        public double rightCurrent = 0.0;
        public double leftAppliedVoltage = 0.0;
        public double rightAppliedVoltage = 0.0;
    }

    public void intake();

    public void outtake();

    public void setZero();

    public boolean isStalling();

    public void passiveIntake();

    public void updateStates(CoralIntakeIOStates states);
}