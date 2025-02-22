package frc.robot.subsystems.coralWrist;

import frc.robot.CommandFactory.ScoringLevel;

import java.util.function.Supplier;

import edu.wpi.first.units.measure.Angle;

public class CoralWristPlacebo implements CoralWristIO {

    @Override
    public void setGoalAngle(Supplier<ScoringLevel> scoringLevel) {
    }

    @Override
    public void setWristSpeed(double speed) {
    }

    @Override
    public void updateStates(CoralWristIOStates states) {
    }

    @Override
    public boolean atGoal() {
        return true;
    }

    @Override
    public void periodic() {
    }

    // @Override
    // public void setGoalStateTrapezoid(Angle angle) {
    // }
}
