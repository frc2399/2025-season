package frc.robot.subsystems.coralWrist;

import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.units.measure.Angle;

public class CoralWristPlacebo implements CoralWristIO {

    @Override
    public void goToSetpoint(Angle angle) {
    }

    @Override
    public void setWristSpeed(double speed) {
    }

    @Override
    public void updateStates(CoralWristIOStates states) {
    }

    @Override
    public void periodic() {}

    @Override
    public void setGoalStateTrapezoid(Angle angle) {}
}
