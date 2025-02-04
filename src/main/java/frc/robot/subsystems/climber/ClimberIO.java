package frc.robot.subsystems.climber;

public interface ClimberIO {
    
    public double getAngle();

    public void setAngle(double desiredPosition);

    public void set(double speed);
}
