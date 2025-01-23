package frc.robot.subsystems.elevator;

public class ElevatorPlacebo implements ElevatorIO {
    
    @Override
    public void setSpeed(double speed) {}

    @Override
    public void setPosition(double position) {}

    @Override
    public void setEncoderPosition(double position) {}

    @Override
    public double getVelocity() {
        return 0.0;
    }

    @Override
    public double getPosition() {
        return 0.0;
    }

    public void setPercentOutput(double percentOutput) {}

    @Override
    public void updateStates(ElevatorIOInputs inputs) {}
}
