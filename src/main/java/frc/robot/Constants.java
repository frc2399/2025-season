// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;

public final class Constants {

  public static class MotorIdConstants {
    public static final int FRONT_LEFT_DRIVING_CAN_ID = 11;
    public static final int REAR_LEFT_DRIVING_CAN_ID = 21;
    public static final int FRONT_RIGHT_DRIVING_CAN_ID = 31;
    public static final int REAR_RIGHT_DRIVING_CAN_ID = 41;

    public static final int FRONT_LEFT_TURNING_CAN_ID = 12;
    public static final int REAR_LEFT_TURNING_CAN_ID = 22;
    public static final int FRONT_RIGHT_TURNING_CAN_ID = 32;
    public static final int REAR_RIGHT_TURNING_CAN_ID = 42;

    public static final int GYRO_CAN_ID = 3;
    
    // alpha
    public static final int CORAL_ALPHA_INTAKE_LEFT_CAN_ID = 7;
    public static final int CORAL_ALPHA_INTAKE_RIGHT_CAN_ID = 5;
    public static final int CORAL_ALPHA_INTAKE_WRIST_CAN_ID = 13;
    public static final int LEFT_ALPHA_ELEVATOR_MOTOR_ID = 17;
    public static final int RIGHT_ALPHA_ELEVATOR_MOTOR_ID = 15;
  
    // beta
    public static final int CORAL_BETA_WRIST_CAN_ID = 46;
    public static final int CORAL_BETA_INTAKE_CAN_ID = 47;
    public static final int LEFT_BETA_ELEVATOR_CAN_ID = 16;
    public static final int RIGHT_BETA_ELEVATOR_CAN_ID = 17;
    public static final int ALGAE_BETA_INTAKE_CAN_ID = 37;
    public static final int ALGAE_BETA_WRIST_CAN_ID = 36;
    public static final int LEFT_CLIMBER_CAN_ID = 26;
    public static final int RIGHT_CLIMBER_CAN_ID = 27; 
  }

  public static class SensorIdConstants {

  }

  public static class MotorConstants {
    public static final Current NEO550_CURRENT_LIMIT = Amps.of(20);
    public static final Current NEO_CURRENT_LIMIT = Amps.of(50);
    public static final Current VORTEX_CURRENT_LIMIT = Amps.of(60);
    public static final AngularVelocity NEO550_FREE_SPEED = RPM.of(11000);
    public static final AngularVelocity NEO_FREE_SPEED = RPM.of(5676);
    public static final AngularVelocity VORTEX_FREE_SPEED = RPM.of(6784);
  }

  public static class SpeedConstants {
    public static final double MAIN_LOOP_FREQUENCY_HZ = 50;
    public static final double DRIVETRAIN_MAX_SPEED_MPS = 4.8;
    public static final double DRIVETRAIN_MAX_ANGULAR_SPEED_RPS = 2 * Math.PI;

    public static final AngularVelocity ALPHA_CORAL_INTAKE_SPEED = MotorConstants.NEO550_FREE_SPEED.times(0.75);
    public static final AngularVelocity ALPHA_CORAL_OUTTAKE_SPEED = MotorConstants.NEO550_FREE_SPEED.times(-0.50);

    public static final AngularVelocity BETA_CORAL_INTAKE_SPEED = MotorConstants.VORTEX_FREE_SPEED.times(0.75);
    public static final AngularVelocity BETA_CORAL_OUTTAKE_SPEED = MotorConstants.VORTEX_FREE_SPEED.times(-0.5);
  }

  public static class SetpointConstants {
    public static final Angle CORAL_INTAKE_ANGLE = Degrees.of(35);
    public static final Angle CORAL_OUTTAKE_ANGLE = Degrees.of(-30);
    public static final Angle CORAL_TURTLE_ANGLE = Degrees.of(35);
    public static final Angle CORAL_L1_ANGLE = Degrees.of(0);

    public static final Distance ELEVATOR_TURTLE_HEIGHT = Inches.of(0);
    public static final Distance L_ONE_HEIGHT = Meters.of(0.125);
    public static final Distance L_TWO_HEIGHT = Meters.of(0.476);
    public static final Distance L_THREE_HEIGHT = Meters.of(0.849);
    public static final Distance L_FOUR_HEIGHT = Inches.of(49.5);
  }

  public static class DriveControlConstants {
    public static final int DRIVER_CONTROLLER_PORT = 0;
    public static final int OPERATOR_CONTROLLER_PORT = 1;
    public static final double DRIVE_DEADBAND = 0.1;
    public static final boolean FIELD_ORIENTED_DRIVE = true;

    public static final Distance ALPHA_TRACK_WIDTH = Meters.of(0.4954);
    public static final Distance MOZART_TRACK_WIDTH = Inches.of(26 - (2 * 1.75));
    // replace if needed
    public static final Distance BETA_XTRACK_WIDTH = Inches.of(23.807);
    public static final Distance BETA_YTRACK_WIDTH = Inches.of(27.190);

  }

}
