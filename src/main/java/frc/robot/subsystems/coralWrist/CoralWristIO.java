package frc.robot.subsystems.coralWrist;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;

public interface CoralWristIO {
    static class CoralWristIOStates {
        public double wristVelocity = 0.0;
        public double wristAppliedVoltage = 0.0;
        public double wristCurrent = 0.0;
        public double wristRelativeEncoderAngle = 0.0;
        // public double trapezoidProfileGoalAngle = 0.0;
        public double goalAngle = 0.0;
    }

    public void setGoalAngle(Angle angle);

    // taking out motion profiling to see if code works
    // public void setGoalStateTrapezoid(Angle angle);

    public void setWristSpeed(AngularVelocity speed);

    public void setWristSpeedType(AngularVelocity speed);

    public void updateStates(CoralWristIOStates states);

    public void periodic();
}
