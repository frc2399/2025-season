package frc.robot.subsystems.algaeIntake;

public interface AlgaeIntakeIO {

    static class AlgaeIntakeIOStates {
        public double intakeVelocity = 0.0;
        public double leftCurrent = 0.0;
        public double rightCurrent = 0.0;
        public double leftAppliedVoltage = 0.0;
        public double rightAppliedVoltage = 0.0;
    }

    public void setSpeed(double speed);

    public double getVelocity();

    public double getCurrent();

    public void updateStates(AlgaeIntakeIOStates states);
}
