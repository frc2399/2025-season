package frc.robot.subsystems.coralIntake;

import java.util.function.Supplier;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.CommandFactory;
import frc.robot.CommandFactory.Setpoint;
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

    public Command setOuttakeSpeed(Supplier<Setpoint> setpoint) {
        return this.run(() -> {
            io.setOuttakeSpeed(setpoint.get());
            setCoralEntry(false);
        });
    }

    public Command defaultBehavior() {
        return this.run(() -> {
            if (hasCoral) {
                io.passiveIntake();
            } else {
                io.setZero();
            }
        });
    }

    public Command passiveIntakeAuton() {
        return this.runOnce(() -> io.passiveIntakeIgnoringStall());
    }

    public Command intakeToStall() {
        return this.run(() -> {
            if (io.isStalling() || hasCoral) {
                io.passiveIntakeIgnoringStall();
                setCoralEntry(true);
            } else {
                io.intake();
            }
        });
    }

    public void setCoralEntry(Boolean coralState) {
        coralEntry.setBoolean(coralState);
        hasCoral = coralState;
    }

    @Override
    public void periodic() {
        hasCoral = coralEntry.getBoolean(hasCoral);
        io.updateStates(states);
        SmartDashboard.putNumber("coralIntake/velocity", states.velocity);
        SmartDashboard.putNumber("coralIntake/goalVelocity", states.goalVelocity);
        SmartDashboard.putNumber("coralIntake/leftCurrent", states.leftCurrent);
        SmartDashboard.putNumber("coralIntake/rightCurrent", 0);
        SmartDashboard.putNumber("coralIntake/leftAppliedVoltage", states.leftAppliedVoltage);
        SmartDashboard.putNumber("coralIntake/rightAppliedVoltage", 0);
        SmartDashboard.putBoolean("coralIntake/isStalling", io.isStalling());
        SmartDashboard.putBoolean("coralIntake/hasCoral", hasCoral);
    }
}
