package frc.robot.subsystems.climber;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.units.measure.Angle;
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

    public Command goToAngle(Angle goalAngle)
    {
        return this.run(() -> climberIO.setGoalAngle(goalAngle));
    }

    public Command setSpeed(double speed)
    {
        return this.run(() -> climberIO.setSpeed(speed));
    }

    public Command setServoAngle(Angle goalAngle)
    {
        return this.run(() -> climberIO.setServoAngle(goalAngle)); 
    }
}
