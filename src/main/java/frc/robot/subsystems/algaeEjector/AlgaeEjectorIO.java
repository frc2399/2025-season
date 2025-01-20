package frc.robot.subsystems.algaeEjector;

public interface AlgaeEjectorIO {

    static class AlgaeEjectorIOStates {
        public double velocity = 0.0;
        public double leftCurrent = 0.0;
        public double rightCurrent = 0.0;
        public double leftAppliedVoltage = 0.0;
        public double rightAppliedVoltage = 0.0;
    }

    public void setSpeed(double speed);

    public double getVelocity();

    public double getCurrent();

    public void updateStates(AlgaeEjectorIOStates states);
}
