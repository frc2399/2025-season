package frc.robot.subsystems.climber;


public class Climber {
    
    private ClimberIO io;
    
        public Climber(ClimberIO io)
        {
            this.io = io; 
    }

    public double getHeight()
    {
        return io.getHeight(); 
    }

    public void setHeight(double height)
    {
        io.setHeight(height);
    }

    public void extend()
    {
        io.extend();
    }

    public void retract()
    {
        io.retract();
    } 

    public boolean isExtended()
    {
        return io.isExtended();
    }

    public boolean isRetracted()
    {
        return io.isRetracted();
    }

    public void set(double speed)
    {
        io.set(speed);
    }
}
