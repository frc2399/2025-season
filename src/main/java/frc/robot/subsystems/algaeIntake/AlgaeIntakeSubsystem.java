package frc.robot.subsystems.algaeIntake;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.algaeIntake.AlgaeIntakeIO.AlgaeIntakeIOStates;

public class AlgaeIntakeSubsystem extends SubsystemBase {

    private final AlgaeIntakeIOStates states = new AlgaeIntakeIOStates();
    private AlgaeIntakeIO io;

    public AlgaeIntakeSubsystem(AlgaeIntakeIO io) {
        this.io = io;
    }

    public Command setRollerSpeed(double speed) {
        return this.run(() -> Commands.run(() -> io.setRollerSpeed(speed)));
    }

    @Override
    public void periodic() {
        io.updateStates(states);
        SmartDashboard.putNumber("algaeIntake/intakeVelocity", states.intakeVelocity);
        SmartDashboard.putNumber("algaeIntake/leftCurrent", states.leftCurrent);
        SmartDashboard.putNumber("algaeIntake/rightCurrent", states.rightCurrent);
        SmartDashboard.putNumber("algaeIntake/leftAppliedVoltage", states.leftAppliedVoltage);
        SmartDashboard.putNumber("algaeIntake/rightAppliedVoltage", states.rightAppliedVoltage);
    }

}