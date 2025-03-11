package frc.robot.subsystems.algaeIntake;

import edu.wpi.first.units.measure.AngularVelocity;

public class AlgaeIntakePlacebo implements AlgaeIntakeIO {

    @Override
    public void setRollerSpeed(AngularVelocity speed) {
    }

    @Override
    public boolean isStalling() {
        return true;
    }

    @Override
    public void updateStates(AlgaeIntakeIOStates states) {
    }

    @Override
    public void intake() {}

    @Override
    public void outtake() {}

    @Override
    public void passiveIntake() {}

}
