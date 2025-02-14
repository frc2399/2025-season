package frc.robot.subsystems.elevator;

import edu.wpi.first.units.measure.Distance;

public interface ElevatorIO {

    static class ElevatorIOStates {
        public double position = 0.0; 
        public double velocity = 0.0;
        public double appliedVoltageRight = 0.0;
        public double appliedVoltageLeft = 0.0;
        public double goalPosition = 0.0;
        public double intermediateSetpointPosition = 0.0;
        public double current = 0.0;        
    }

    public void resetSetpointsToCurrentPosition(); 
    public void incrementGoalPosition(Distance newGoalPosition);
    public void setGoalPosition(Distance position);
    public void calculateNextIntermediateSetpoint();
    public void setIntermediateSetpoint(Distance position, double velocity);
    public double getEncoderVelocity();
    public double getEncoderPosition();
    public void updateStates(ElevatorIOStates states);
}