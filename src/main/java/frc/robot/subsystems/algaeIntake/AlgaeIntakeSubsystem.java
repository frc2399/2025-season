package frc.robot.subsystems.algaeIntake;

import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.algaeIntake.AlgaeIntakeIO.AlgaeIntakeIOStates;

public class AlgaeIntakeSubsystem extends SubsystemBase {

    private final AlgaeIntakeIOStates states = new AlgaeIntakeIOStates();
    private AlgaeIntakeIO io;
    public boolean hasAlgae = false;

    public AlgaeIntakeSubsystem(AlgaeIntakeIO io) {
        this.io = io;
    }
    public Command intake() {
        return this.run(() -> io.intake()).withName("run algae intake");
    }

    public Command outtake() {
        return this.run(() -> {io.outtake(); hasAlgae= false;}).withName("run algae outtake");
    }
    public Command setRollerSpeed(AngularVelocity speed) {
        return this.run(() -> io.setRollerSpeed(speed));
    }

    public Command intakeToStall() {
        return this.run(
          () -> {
            
                if(io.isStalling() || hasAlgae){
                    io.setRollerSpeed(RPM.of(0));
                    hasAlgae = true;
                }
                else{
                    io.setRollerSpeed(Constants.SpeedConstants.ALGAE_INTAKE_SPEED);

                }
          }
        );
    }
    

    @Override
    public void periodic() {
        io.updateStates(states);
        SmartDashboard.putNumber("algaeIntake/intakeVelocity", states.intakeVelocity);
        SmartDashboard.putNumber("algaeIntake/leftCurrent", states.leftCurrent);
        SmartDashboard.putNumber("algaeIntake/leftAppliedVoltage", states.leftAppliedVoltage);
    }

}