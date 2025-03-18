package frc.robot.subsystems.climber;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

    @Override
    public void periodic() {
        climberIO.updateStates(inputs);
        SmartDashboard.putNumber("climber/climberAngle", inputs.climberAngle);
        SmartDashboard.putNumber("climber/climberVelocity", inputs.climberVelocity);
        //TODO: we can likely take this out - climber is manual only!
        SmartDashboard.putNumber("climber/goalAngle", inputs.climberGoalAngle);
        SmartDashboard.putNumber("climber/servoAngle", inputs.servoAngle);
        //TODO: is this one really necessary?
        SmartDashboard.putNumber("climber/servoVelocity", inputs.servoVelocity);
        SmartDashboard.putNumber("climber/servoGoalAngle", inputs.servoGoalAngle);
    }
}
