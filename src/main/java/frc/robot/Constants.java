// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;

import static edu.wpi.first.units.Units.Centimeters;
import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.units.AngularVelocityUnit;
import edu.wpi.first.units.CurrentUnit;

import static edu.wpi.first.units.Units.*;

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

    public static final int LEFT_ELEVATOR_MOTOR_ID = 17;
    public static final int RIGHT_ELEVATOR_MOTOR_ID = 15;
  }

  public static class SensorIdConstants {

  }

  public static class MotorConstants {
    public static final Current NEO550_CURRENT_LIMIT = Amps.of(20);
    public static final Current NEO_CURRENT_LIMIT = Amps.of(50);
    public static final AngularVelocity NEO_FREE_SPEED = RPM.of(5676);
    public static final AngularVelocity NEO550_FREE_SPEED = RPM.of(11000);
    public static final Current NEO_VORTEX_CURRENT_LIMIT = Amps.of(60);
  }

  public static class SpeedConstants {
    public static final double MAIN_LOOP_FREQUENCY_HZ = 50;

    public static final double DRIVETRAIN_MAX_SPEED_MPS = 4.8;
    public static final double DRIVETRAIN_MAX_ANGULAR_SPEED_RPS = 2 * Math.PI;
  }

  public static class SetpointConstants {
    public static Distance L_ONE_HEIGHT = Inches.of(0); 
    public static Distance L_TWO_HEIGHT = Inches.of(7.5);
    public static Distance L_THREE_HEIGHT = Inches.of(23.25);
    public static Distance L_FOUR_HEIGHT = Inches.of(49.5);
  }

  public static class DriveControlConstants {
    public static final int DRIVER_CONTROLLER_PORT = 0;
    public static final int OPERATOR_CONTROLLER_PORT = 1;
    public static final double DRIVE_DEADBAND = 0.05;
    public static final boolean FIELD_ORIENTED_DRIVE = true;

    public static final Distance ALPHA_TRACK_WIDTH = Meters.of(0.4954);
    public static final Distance MOZART_TRACK_WIDTH = Inches.of(26 - (2 * 1.75));
  }

}