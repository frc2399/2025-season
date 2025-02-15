package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.subsystems.climber.ClimberIO.ClimberIOInputs;

public class ClimberSubsystem extends SubsystemBase {
    
    private ClimberIO climberIO;
    private final ClimberIOInputs inputs = new ClimberIOInputs();
    
    public ClimberSubsystem(ClimberIO io)
    {
            this.climberIO = io; 
    }

    public double getAngle()
    {
        return climberIO.getAngle(); 
    }

    public Command setAngle(double height)
    {
        return this.run(() -> climberIO.setGoalAngle(height));
    }

    //TO DO: return cmd 
    public Command set(double speed)
    {
        return this.run(() -> climberIO.setSpeed(speed));
    }
}
