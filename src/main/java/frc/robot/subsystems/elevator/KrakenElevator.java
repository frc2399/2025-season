package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import static edu.wpi.first.units.Units.*;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants.MotorConstants;
import frc.robot.Constants.MotorIdConstants;

public class KrakenElevator implements ElevatorIO {

    public static final class ElevatorHardwareConstants {
        private static final Distance METERS_PER_REVOLUTION = Inches.of(0.67); // (1/9)(1.92 * pi)
        private static final Distance ALLOWED_SETPOINT_ERROR = Inches.of(.25);
        private static final LinearVelocity MAX_VEL = MetersPerSecond.of(0.8);
        private static final LinearAcceleration MAX_ACCEL = MetersPerSecondPerSecond.of(0.4);
        private static final Voltage P_VALUE = Volts.of(2.0);
        private static final Voltage I_VALUE = Volts.of(0);
        private static final Voltage D_VALUE = Volts.of(0);
        private static final Voltage FEEDFORWARD_VALUE = Volts.of(1.0 / 917);
        private static final Voltage ARBITRARY_FF_GRAVITY_COMPENSATION = Volts.of(0.28); // calculated using recalc
        private static final double OUTPUT_RANGE_MIN_VALUE = -1;
        private static final double OUTPUT_RANGE_MAX_VALUE = 1;
        private static final double P_VALUE_VELOCITY = 0.0001;
        private static final double I_VALUE_VELOCITY = 0;
        private static final double D_VALUE_VELOCITY = 0;
        private static final Distance MAX_ELEVATOR_HEIGHT = Inches.of(34.25); // inches
    }

    private TalonFX elevatorRightMotorFollower, elevatorLeftMotorLeader;
    private TalonFXConfigurator globalMotorConfig, rightMotorConfigFollower, leftMotorConfigLeader;
    private SparkClosedLoopController leftClosedLoopController;
    private RelativeEncoder leftEncoder;
    private double goalPosition;

    public KrakenElevator() {

    }

    @Override
    public void disableElevator() {
    }

    @Override
    public void setSpeed(double speed) {
       
    }

    @Override
    public void setGoalPosition(double desiredPosition) {

    }

    @Override
    public void setEncoderPosition(double position) {
    }

    @Override
    public double getEncoderVelocity() {
        return 0; 
    }

    @Override
    public double getEncoderPosition() {
        return 0; 
    }

    @Override
    public void setPercentOutput(double percentOutput) {
    }

    @Override
    public void updateStates(ElevatorIOInputs inputs) {

    }
}
