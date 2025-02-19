package frc.robot.subsystems.elevator;

import edu.wpi.first.units.measure.Distance;
import frc.robot.CommandFactory.ScoringLevel;

public class ElevatorPlacebo implements ElevatorIO {

    @Override
    public void resetSetpointsToCurrentPosition() {
    }

    public void incrementGoalPosition(Distance changeInGoalPosition) {
    }

    @Override
    public void setGoalPosition(Distance position) {
    }

    @Override
    public void calculateNextIntermediateSetpoint() {
    }

    @Override
    public void setIntermediateSetpoint(Distance position, double velocity) {
    }

    @Override
    public double getEncoderVelocity() {
        return 0.0;
    }

    @Override
    public double getEncoderPosition() {
        return 0.0;
    }

    @Override
    public boolean willCrossCronchZone(ScoringLevel scoringLevel) {
        return false;
    }

    @Override
    public void updateStates(ElevatorIOStates states) {
    }
}
