package frc.robot.subsystems.algaeIntake;

public interface AlgaeIntakeIO {

    static class AlgaeIntakeIOStates {
        public double intakeVelocity = 0.0;
        public double leftCurrent = 0.0;
        public double leftAppliedVoltage = 0.0;
    }

    public void setRollerSpeed(double speed);

    public void updateStates(AlgaeIntakeIOStates states);
}
