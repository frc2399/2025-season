package frc.robot.subsystems.coralIntake;

import edu.wpi.first.units.measure.Angle;

public interface CoralIntakeIO
{
   static class CoralIntakeIOStates
   {
       public double velocity = 0.0;
       public double leftCurrent = 0.0;
       public double rightCurrent = 0.0;
       public double wristCurrent = 0.0;
       public double leftAppliedVoltage = 0.0;
       public double rightAppliedVoltage = 0.0;
       public double wristAppliedVoltage = 0.0;
   }
   
   public void setSpeed(double speed);
   public void goToSetpoint(Angle angle);
   public double getVelocity();
   public double getCurrent();
   public void updateStates(CoralIntakeIOStates states);
   public void setGravityCompensation();
}