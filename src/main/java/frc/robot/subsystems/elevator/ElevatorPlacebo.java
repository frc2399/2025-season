package frc.robot.subsystems.elevator;

public class ElevatorPlacebo implements ElevatorIO {

    @Override
    public void disableElevator() {}
    
    @Override
    public void setSpeed(double speed) {}

    @Override
    public void setPositionPID(double position) {}

    @Override
    public void setPositionMotionProfiling(double position) {}

    @Override  
    public void calculateNextSetpoint() {}

    @Override
    public void setEncoderPosition(double position) {}

    @Override
    public void setSetpointState(double position, double velocity) {}

    @Override
    public double getVelocity() {
        return 0.0;
    }

    @Override
    public double getCurrentPosition() {
        return 0.0;
    }

    public void setPercentOutput(double percentOutput) {}

    @Override
    public void updateStates(ElevatorIOInputs inputs) {}
}
