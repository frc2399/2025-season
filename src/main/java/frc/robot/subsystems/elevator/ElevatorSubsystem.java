package frc.robot.subsystems.elevator;

import edu.wpi.first.units.measure.Distance;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.elevator.ElevatorIO.ElevatorIOInputs;

public class ElevatorSubsystem extends SubsystemBase{
    
    private final ElevatorIO elevatorIO;
    private final ElevatorIOInputs inputs = new ElevatorIOInputs();
    public boolean profiledPIDEnabled = false;
    private double setpoint; 
    private final Distance HEIGHT_TOLERANCE = Inches.of(0.5);
    private double JOYSTICK_INPUT_TO_CHANGE_IN_POSITION_CONVERSION_FACTOR = 0.001; 

    public static final class ElevatorConstants {}

    public ElevatorSubsystem(ElevatorIO elevatorIO) {
        this.elevatorIO = elevatorIO;
        elevatorIO.setEncoderPosition(0);
        elevatorIO.setSetpointState(0, 0);
    }

    public Command disableElevator() {
        return this.run(() ->elevatorIO.disableElevator());
    }

    public Command setEncoderPositionCommand(double position) {
        return this.run(() -> elevatorIO.setEncoderPosition(position));
    }

    //pid command that is seperate from motion profiling
    public Command goToSetPointCommand(double position) {
        return this.runOnce(() -> elevatorIO.setPositionMotionProfiling(position));
    }

    //motion profile command that is seperate from PID
    public Command goToSetpointCmdMotionProfling(double position) {
        return this.runOnce(() -> {
            elevatorIO.setPositionMotionProfiling(position); 
            profiledPIDEnabled = true;
            setpoint = position; 
        });
    }

    public Command setSpeedCommand(double speed) {
        return this.run(() -> elevatorIO.setSpeed(speed));
    }

    public Command setPercentOutputCommand(double percentOutput) {
        return this.runOnce(() -> elevatorIO.setPercentOutput(percentOutput));
    }

    public boolean atGoal(){
        return (Math.abs(setpoint - elevatorIO.getEncoderPosition()) <= HEIGHT_TOLERANCE.in(Meters));
    }

    public Command keepElevatorAtCurrentPosition() {
        return this.run(() -> elevatorIO.setGoalPosition(elevatorIO.getEncoderPosition()));
    }


    public Command incrementGoalPosition(double changeInGoalPosition)
    {
        return this.run(()-> elevatorIO.incrementGoalPosition(changeInGoalPosition));
    }


    @Override
    public void periodic() {
        if (profiledPIDEnabled) {
            elevatorIO.calculateNextSetpoint();
        }

        elevatorIO.updateStates(inputs);
        SmartDashboard.putNumber("Elevator/position", inputs.position);
        SmartDashboard.putNumber("Elevator/velocity", inputs.velocity);
        SmartDashboard.putNumber("Elevator/appliedVoltageRight", inputs.appliedVoltageRight);
        SmartDashboard.putNumber("Elevator/appliedVoltageLeft", inputs.appliedVoltageLeft);
        SmartDashboard.putNumber("Elevator/positionSetPoint", inputs.positionSetPoint);
        SmartDashboard.putNumber("Elevator/goalStatePosition", inputs.goalStatePosition);
        SmartDashboard.putNumber("Elevator/output current", inputs.current);
        SmartDashboard.putNumber("Elevator/setpointStatePosition", inputs.setpointStatePosition);
    }

}
 