package frc.robot.subsystems.coralIntake;

public class CoralIntakePlacebo implements CoralIntakeIO {
   public void setSpeed(double speed) {
       speed = 0;
   }

   public double getVelocity() {
       return 0.0;
   }

   public double getCurrent() {
       return 0.0;
   }

   public void updateStates(CoralIntakeIOStates states) {
   }
}
