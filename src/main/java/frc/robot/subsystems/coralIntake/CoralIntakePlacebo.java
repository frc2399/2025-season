package frc.robot.subsystems.coralIntake;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.CommandFactory.Setpoint;

public class CoralIntakePlacebo implements CoralIntakeIO {

    @Override
    public void intake() {
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
    public void passiveIntake() {
    }

    public void passiveIntakeIgnoringStall() 
    {
        
    }

    @Override
    public void updateStates(CoralIntakeIOStates states) {
    }

    @Override
    public void setVoltage(Voltage volts) {
    }

    @Override
        public double getAppliedVoltage() {
                return 0;
        }

        @Override
        public double getPosition() {
                return 0;
        }

        @Override
        public AngularVelocity getAngularVelocity() {
                return RotationsPerSecond.of(0);
        }
}