package frc.robot.subsystems.coralIntake;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.CommandFactory;
import frc.robot.subsystems.coralIntake.CoralIntakeIO.CoralIntakeIOStates;

public class CoralIntakeSubsystem extends SubsystemBase {
    private final CoralIntakeIOStates states = new CoralIntakeIOStates();
    private CoralIntakeIO io;
    public boolean hasCoral = false;
    private final NetworkTableEntry coralEntry = CommandFactory.scoringStateTables.getEntry("hasCoral");

    public CoralIntakeSubsystem(CoralIntakeIO io) {
        this.io = io;
    }

    public Command intake() {
        return this.run(() -> io.intake()).withName("run coral intake");
    }

    public Command outtake() {
        return this.run(() -> {
            io.outtake();
            if (hasCoral) {
                setCoralEntry(false);
            }
        });
    }

    public Command setZero() {
        return this.run(() -> io.setZero()).withName("coral intake default");
    }

    public Command intakeToStall() {
        return this.run(() -> {
            if (io.isStalling() || hasCoral) {
                io.setZero();
                setCoralEntry(true);
            } else {
                io.intake();
            }
        });
    }

    public Command passiveIntakeCommand() {
        return this.runOnce(() -> { 
            io.passiveIntake();
        });
    }

    public void setCoralEntry(Boolean coralState) {
        coralEntry.setBoolean(coralState);
        hasCoral = coralState;
    }

    @Override
    public void periodic() {
        io.updateStates(states);
        SmartDashboard.putNumber("coralIntake/velocity", states.velocity);
        SmartDashboard.putNumber("coralIntake/goalVelocity", states.goalVelocity);
        SmartDashboard.putNumber("coralIntake/leftCurrent", states.leftCurrent);
        SmartDashboard.putNumber("coralIntake/rightCurrent", states.rightCurrent);
        SmartDashboard.putNumber("coralIntake/leftAppliedVoltage", states.leftAppliedVoltage);
        SmartDashboard.putNumber("coralIntake/rightAppliedVoltage", states.rightAppliedVoltage);
    }
}
