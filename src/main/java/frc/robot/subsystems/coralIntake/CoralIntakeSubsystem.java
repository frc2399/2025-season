package frc.robot.subsystems.coralIntake;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.coralIntake.CoralIntakeIO.CoralIntakeIOStates;

public class CoralIntakeSubsystem extends SubsystemBase {
    private final CoralIntakeIOStates states = new CoralIntakeIOStates();
    private CoralIntakeIO io;

    public CoralIntakeSubsystem(CoralIntakeIO io) {
        this.io = io;
    }

    public Command setRollerSpeed(AngularVelocity speed) {
        return this.run(() -> io.setRollerSpeed(speed));
    }

    public Command setSpeedType(AngularVelocity speed) {
        return this.run(() -> io.setSpeedType(speed));
    }

    @Override
    public void periodic() {
        io.updateStates(states);
        SmartDashboard.putNumber("coralIntake/velocity", states.velocity);
        SmartDashboard.putNumber("coralIntake/leftCurrent", states.leftCurrent);
        SmartDashboard.putNumber("coralIntake/rightCurrent", states.rightCurrent);
        SmartDashboard.putNumber("coralIntake/leftAppliedVoltage", states.leftAppliedVoltage);
        SmartDashboard.putNumber("coralIntake/rightAppliedVoltage", states.rightAppliedVoltage);
    }
}
