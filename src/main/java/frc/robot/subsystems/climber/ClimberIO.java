package frc.robot.subsystems.climber;

public interface ClimberIO {
    
    public double getHeight();

    public void setHeight(double height);
    
    public void extend();

    public void retract(); 

    public boolean isExtended();

    public boolean isRetracted();

    public void set(double speed);
}
