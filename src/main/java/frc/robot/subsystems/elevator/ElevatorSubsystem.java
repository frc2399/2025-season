package frc.robot.subsystems.elevator;

import edu.wpi.first.units.measure.Distance;
<<<<<<< HEAD
import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.MathUtil;
=======
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import java.util.function.DoubleSupplier;

>>>>>>> main
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.elevator.ElevatorIO.ElevatorIOStates;;

public class ElevatorSubsystem extends SubsystemBase{
    
    private final ElevatorIO elevatorIO;
<<<<<<< HEAD
    private final ElevatorIOInputs inputs = new ElevatorIOInputs();
    private double setpoint;
    private final Distance HEIGHT_TOLERANCE = Inches.of(0.5);
=======
    private final ElevatorIOStates states = new ElevatorIOStates();
    public boolean profiledPIDEnabled = false;
    private double goalSetpoint; 
    private final Distance HEIGHT_TOLERANCE = Inches.of(0.5);
    private double JOYSTICK_INPUT_TO_CHANGE_IN_POSITION_CONVERSION_FACTOR = 0.002; 
>>>>>>> main

    public static final class ElevatorConstants {}

    public ElevatorSubsystem(ElevatorIO elevatorIO) {
        this.elevatorIO = elevatorIO;
        elevatorIO.setIntermediateSetpoint(Meters.of(0), 0);
    }


    public Command goToGoalSetpointCmd(Distance position) {
        return this.runOnce(() -> {
            elevatorIO.setGoalPosition(position); 
            profiledPIDEnabled = true;
            goalSetpoint = position.in(Meters); 
        });
    }

    public boolean atGoal(){
        return (Math.abs(goalSetpoint - elevatorIO.getEncoderPosition()) <= HEIGHT_TOLERANCE.in(Meters));
    }

<<<<<<< HEAD
    public Command goToSetPointCommand(double position) {
        return this.runOnce(() -> { 
            elevatorIO.setGoalPosition(position);
            setpoint = position;
        });
    }

    public boolean atGoal() {
        boolean isAtHeight = MathUtil.isNear(setpoint, elevatorIO.getEncoderPosition(), HEIGHT_TOLERANCE.in(Meters));
        SmartDashboard.putNumber("Elevator/current position", elevatorIO.getEncoderPosition());
        SmartDashboard.putNumber("Elevator/setpoint", setpoint);
        SmartDashboard.putBoolean("Elevator/at goal", isAtHeight);
        return isAtHeight;
    }

    public Command atGoalCommand() {
        return Commands.waitUntil(() -> atGoal());
=======
    public Command incrementGoalPosition(Distance changeInGoalPosition)
    {
        profiledPIDEnabled = true;
        return this.run(()-> elevatorIO.incrementGoalPosition(changeInGoalPosition));
>>>>>>> main
    }

    public double getCurrentPosition()
    {
        return elevatorIO.getEncoderPosition();
    }


    @Override
    public void periodic() {
        if (!profiledPIDEnabled) {
            elevatorIO.resetSetpointsToCurrentPosition();
        }
        elevatorIO.calculateNextIntermediateSetpoint();

        elevatorIO.updateStates(states);
        SmartDashboard.putNumber("Elevator/position", states.position);
        SmartDashboard.putNumber("Elevator/velocity", states.velocity);
        SmartDashboard.putNumber("Elevator/appliedVoltageRight", states.appliedVoltageRight);
        SmartDashboard.putNumber("Elevator/appliedVoltageLeft", states.appliedVoltageLeft);
        SmartDashboard.putNumber("Elevator/goalStatePosition", states.goalPosition);
        SmartDashboard.putNumber("Elevator/output current", states.current);
        SmartDashboard.putNumber("Elevator/intermediate setpoint position", states.intermediateSetpointPosition);
        SmartDashboard.putBoolean("Elevator/profiled PID enabled", profiledPIDEnabled); 
    }

}
 