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
    private double JOYSTICK_INPUT_TO_CHANGE_IN_POSITION_CONVERSION_FACTOR = 0.002; 

    public static final class ElevatorConstants {}

    public ElevatorSubsystem(ElevatorIO elevatorIO) {
        this.elevatorIO = elevatorIO;
        elevatorIO.setEncoderPosition(0);
        elevatorIO.setSetpointState(0, 0);
    }

    public void disableElevator() {  
        elevatorIO.disableElevator();
    }

    public void enableElevator()
    {
        elevatorIO.enableElevator();
    }

    public Command setEncoderPositionCommand(double position) {
        return this.run(() -> elevatorIO.setEncoderPosition(position));
    }

    //pid command that is seperate from motion profiling
    public Command goToSetPointCommand(double position) {
        return this.runOnce(() -> elevatorIO.setGoalPosition(position));
    }

    //motion profile command that is seperate from PID
    public Command goToSetpointCmdMotionProfling(double position) {
        return this.runOnce(() -> {
            elevatorIO.setGoalPosition(position); 
            profiledPIDEnabled = true;
            setpoint = position; 
        });
    }

    public boolean atGoal(){
        return (Math.abs(setpoint - elevatorIO.getEncoderPosition()) <= HEIGHT_TOLERANCE.in(Meters));
    }

    public Command incrementGoalPosition(double changeInGoalPosition)
    {
        profiledPIDEnabled = true;
        return this.run(()-> elevatorIO.incrementGoalPosition(changeInGoalPosition));
    }

    public double getCurrentPosition()
    {
        return elevatorIO.getEncoderPosition();
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
 