package frc.robot.subsystems.coralIntake;

import frc.robot.CommandFactory.Setpoint;

public class CoralIntakePlacebo implements CoralIntakeIO {

    @Override
    public void intake() {
    }

    @Override
    public void outtake() {
    }

    @Override
    public void setOuttakeSpeed(Setpoint setpoint) {
    }

    @Override
    public void setZero() {
    }

    @Override
    public boolean isStalling() {
        return true;
    }

    @Override
    public void updateStates(CoralIntakeIOStates states) {
    }
}