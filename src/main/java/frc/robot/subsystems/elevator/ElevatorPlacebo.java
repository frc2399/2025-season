package frc.robot.subsystems.elevator;

public class ElevatorPlacebo implements ElevatorIO {
    
    @Override
    public void disableElevator() {}

    @Override
    public void setSpeed(double speed) {}

    @Override
    public void setGoalPosition(double position) {}

    @Override
    public void setEncoderPosition(double position) {}

    @Override
    public double getEncoderVelocity() {
        return 0.0;
    }

    @Override
    public double getEncoderPosition() {
        return 0.0;
    }

    public void setPercentOutput(double percentOutput) {}

    @Override
    public void updateStates(ElevatorIOInputs inputs) {}
}
