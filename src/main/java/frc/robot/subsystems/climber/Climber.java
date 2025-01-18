package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
    
    private ClimberIO io;
    
    public Climber(ClimberIO io)
    {
            this.io = io; 
    }

    public double getHeight()
    {
        return io.getHeight(); 
    }

    public Command setHeight(double height)
    {
        return this.run(() -> io.setHeight(height));
    }

    public Command extend()
    {
        return this.run(() -> io.extend());
    }

    public Command retract()
    {
        return this.run(() -> io.retract());
    } 

    public boolean isExtended()
    {
        return io.isExtended();
    }

    public boolean isRetracted()
    {
        return io.isRetracted();
    }

    //TO DO: return cmd 
    public Command set(double speed)
    {
        return this.run(() -> io.set(speed));
    }
}
