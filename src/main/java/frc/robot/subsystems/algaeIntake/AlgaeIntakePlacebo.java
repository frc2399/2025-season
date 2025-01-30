package frc.robot.subsystems.algaeIntake;

public class AlgaeIntakePlacebo implements AlgaeIntakeIO {

    @Override
    public void setSpeed(double speed) {
        speed = 0;
    }

    @Override
    public void updateStates(AlgaeIntakeIOStates states) {
    }

    @Override
    public double getVelocity() {
        return 0.0;
    }

    @Override
    public double getCurrent() {
        return 0.0;
    }
}
