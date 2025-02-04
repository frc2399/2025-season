package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
    
    private ClimberIO io;
    
    public Climber(ClimberIO io)
    {
            this.io = io; 
    }

    public double getAngle()
    {
        return io.getAngle(); 
    }

    public Command setAngle(double height)
    {
        return this.run(() -> io.setAngle(height));
    }

    //TO DO: return cmd 
    public Command set(double speed)
    {
        return this.run(() -> io.set(speed));
    }
}
