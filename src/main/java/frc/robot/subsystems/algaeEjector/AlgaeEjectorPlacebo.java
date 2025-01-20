package frc.robot.subsystems.algaeEjector;

public class AlgaeEjectorPlacebo implements AlgaeEjectorIO {

    @Override
    public void setSpeed(double speed) {
        speed = 0;
    }

    @Override
    public void updateStates(AlgaeEjectorIOStates states) {
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
