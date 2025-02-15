package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.subsystems.climber.ClimberIO.ClimberIOInputs;
import frc.robot.subsystems.climber.ClimberIO.ClimberIOInputs;

public class ClimberSubsystem extends SubsystemBase {
    
    private ClimberIO climberIO;
    private final ClimberIOInputs inputs = new ClimberIOInputs();
    
    public ClimberSubsystem(ClimberIO climberIO)
    {
            this.climberIO = climberIO; 
            //add a line setting the climber to its initial position 
    }

    public double getAngle()
    {
        return climberIO.getAngle(); 
    }

    public double getVelocity()
    {
        return climberIO.getVelocity(); 
    }

    public Command goToAngle(double height)
    {
        return this.run(() -> climberIO.setGoalAngle(height));
    }

    public Command setSpeed(double speed)
    {
        return this.run(() -> climberIO.setSpeed(speed));
    }
}
