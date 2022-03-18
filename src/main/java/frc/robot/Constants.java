// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean constants. This
 * class should not be used for any other purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes) wherever the constants are needed, to
 * reduce verbosity.
 */
public final class Constants {

  /**
   * Stores constants related to autonomous routines.
   */
  public static final class AutonConstants {
    public static final double SIMPLE_AUTON_SPEED = 0.5; // WEEK0
    public static final double SIMPLE_AUTON_RUNTIME = 4.0; // WEEK0
    public static final double MEDIUM_AUTON_OUTTAKE_RUNTIME = 2.0; // PLACEHOLDER

    public static final double KS_VOLTS = 0.0; // PLACEHOLDER
    public static final double KV_VOLT_SECONDS_PER_METER = 0.0; // PLACEHOLDER
    public static final double KA_VOLT_SECONDS_SQUARED_PER_METER = 0.0; // PLACEHOLDER

    public static final double KP_DRIVE_VEL = 0.0; // PLACEHOLDER

    public static final double K_TRACK_WIDTH_METERS = 0.69;
    public static final double K_MAX_SPEED_METERS_PER_SECOND = 3;
    public static final double K_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED = 3;
    public static final double K_RAMSETE_B = 2;
    public static final double K_RAMSETE_ZETA = 0.7;
  }

  /**
   * Stores constants related to driver controls, SmartDashboard and other IO (Input/Output).
   */
  public static final class IOConstants {
    public static final int DRIVER_JOYSTICK_INDEX = 0;

    public static final class DriverButtons {
      public static final int SLOWDOWN = 1; // PLACEHOLDER
    }

    public static final class DriverAxes {
      public static final int LEFT = 1; // PLACEHOLDER
      public static final int RIGHT = 3; // PLACEHOLDER
    }

    public static final int OPERATOR_JOYSTICK_INDEX = 1;

    public static final class OperatorButtons {
      public static final int INTAKE = 0; // PLACEHOLDER

      public static final int OUTTAKE = 2; // WEEK0

      public static final int TOWER = 6; // WEEK0

      public static final int CLIMBER_UP = 4; // WEEK0
      public static final int CLIMBER_DOWN = 1; // WEEK0
    }
  }

  /**
   * Stores constants related to the DriveBase.
   */
  public static final class DriveConstants {
    public static final double SLOWDOWN_PERCENT = 0.5;

    public static final class DriveMotorCANIDs {
      public static final int LEFT_FRONT = 54; // WEEK0
      public static final int LEFT_BACK = 55; // WEEK0
      public static final int RIGHT_FRONT = 52; // WEEK0
      public static final int RIGHT_BACK = 53; // WEEK0
    }

    public static final int CURRENT_LIMIT = 60;
    public static final int TRIGGER_THRESHOLD = 60;
    public static final int TRIGGER_THRESHOLD_TIME = 2;
  }

  /**
   * Stores constants related to the Intake.
   */
  public static final class IntakeConstants {
    public static final double SPEED = 1.0; // PLACEHOLDER

    public static final double SOLENOID_DELAY_TIME = 1.0; // PLACEHOLDER

    public static final class IntakeSolenoidChannels {
      public static final int TOP_FORWARD = 0; // PLACEHOLDER
      public static final int TOP_REVERSE = 1; // PLACEHOLDER
      public static final int BOTTOM_FORWARD = 2; // PLACEHOLDER
      public static final int BOTTOM_REVERSE = 3; // PLACEHOLDER
    }

    public static final int MOTOR_ID = 0; // PLACEHOLDER

    public static final int CURRENT_LIMIT = 40;
    public static final int TRIGGER_THRESHOLD = 40;
    public static final int TRIGGER_THRESHOLD_TIME = 2;
  }

  /**
   * Stores constants related to the Outtake.
   */
  public static final class OuttakeConstants {
    public static final double SPEED = 0.5; // PLACEHOLDER

    public static final class OuttakeMotorCANIDs {
      public static final int TOP = 1; // PLACEHOLDER
      public static final int BOTTOM = 2; // PLACEHOLDER
    }

    public static final int CURRENT_LIMIT = 40;
  }

  /**
   * Stores constants related to the Tower.
   */

  public static final class TowerConstants {
    public static final double SPEED = 1.0; // PLACEHOLDER

    public static final class TowerMotorCANIDs {
      public static final int TOP_LEFT = 4; // PLACEHOLDER
      public static final int BOTTOM_LEFT = 5; // PLACEHOLDER
      public static final int TOP_RIGHT = 6; // PLACEHOLDER
      public static final int BOTTOM_RIGHT = 7; // PLACEHOLDER
    }

    public static final int CURRENT_LIMIT = 25;
  }

  /**
   * Stores constants related to the Climber.
   */
  public static final class ClimberConstants {
    public static final double SPEED = 1.0; // WEEK0

    public static final class ClimberMotorCANIDs {
      public static final int LEFT = 50; // PLACEHOLDER
      public static final int RIGHT = 51; // PLACEHOLDER
    }

    public static final int CURRENT_LIMIT = 85;
    public static final int TRIGGER_THRESHOLD = 100;
    public static final double TRIGGER_THRESHOLD_TIME = 0.5;
  }

  /**
   * Stores constants related to the Pneumatics.
   */
  public final class PneumaticsConstants {
    public static final double DEFAULT_MIN_PRESSURE = 90.0; // PLACEHOLDER
    public static final double DEFAULT_MAX_PRESSURE = 115.0; // PLACEHOLDER
  }

}
