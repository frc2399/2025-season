// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

public final class Constants {

  public static class MotorIdConstants{
    public static final int FRONT_LEFT_DRIVING_CAN_ID = 11;
    public static final int REAR_LEFT_DRIVING_CAN_ID = 21;
    public static final int FRONT_RIGHT_DRIVING_CAN_ID = 31;
    public static final int REAR_RIGHT_DRIVING_CAN_ID = 41;

    public static final int FRONT_LEFT_TURNING_CAN_ID = 12;
    public static final int REAR_LEFT_TURNING_CAN_ID = 22;
    public static final int FRONT_RIGHT_TURNING_CAN_ID = 32;
    public static final int REAR_RIGHT_TURNING_CAN_ID = 42;

    public static final int GYRO_CAN_ID = 3;

    public static final int CLIMBER_LEFT_CAN_ID = 0;
    public static final int CLIMBER_RIGHT_CAN_ID = 1;
  }
  
  public static class SensorIdConstants {

  }

  public static class MotorConstants {
    public static final int NEO550_CURRENT_LIMIT = 20;
    public static final int NEO_CURRENT_LIMIT = 50;
    public static final int NEO_FREE_SPEED_RPM = 5676;
    public static final double NEO_FREE_SPEED_RPS = NEO_FREE_SPEED_RPM / 60;
    // MPS = (GearRatio * 2πr * RPM) / 60
    public static final int NEO550_FREE_SPEED_RPM = 11000;
  }

  public static class SpeedConstants {
    public static final double MAIN_LOOP_FREQUENCY_HZ = 50;

    public static final double DRIVETRAIN_MAX_SPEED_MPS = 4.8;
    public static final double DRIVETRAIN_MAX_ANGULAR_SPEED_RPS = 2 * Math.PI;
  }

  public static class SetpointConstants {

  }

  public static class DriveControlConstants {
    public static final int DRIVER_CONTROLLER_PORT = 0;
    public static final int OPERATOR_CONTROLLER_PORT = 1;
    public static final double DRIVE_DEADBAND = 0.05;
    public static final boolean FIELD_ORIENTED_DRIVE = true;
  }

}
