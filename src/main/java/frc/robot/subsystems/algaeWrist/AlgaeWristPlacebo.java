package frc.robot.subsystems.algaeWrist;

public class AlgaeWristPlacebo implements AlgaeWristIO {

    @Override
    public void setSpeed(double speed) {
        speed = 0;
    }

    @Override
    public void updateStates(AlgaeWristIOStates states) {
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
