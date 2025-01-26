package frc.robot.subsystems.coralIntake;

import edu.wpi.first.units.measure.Angle;

public class CoralIntakePlacebo implements CoralIntakeIO {
   public void setRollerSpeed(double speed) {
       speed = 0;
   }

   public void setWristSpeed(double speed) {}

   public void setGravityCompensation() {}

   public double getVelocity() {
       return 0.0;
   }

   public double getCurrent() {
       return 0.0;
   }

   public void goToSetpoint(Angle angle) {
   }

   public void updateStates(CoralIntakeIOStates states) {
   }
}
