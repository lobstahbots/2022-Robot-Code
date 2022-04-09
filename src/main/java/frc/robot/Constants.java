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
    public static final double SIMPLE_AUTON_SPEED = 0.7;
    public static final double SIMPLE_AUTON_RUNTIME = 3.0;
    public static final double MEDIUM_AUTON_OUTTAKE_RUNTIME = 3.0;
  }

  /**
   * Stores constants related to driver controls, SmartDashboard and other IO (Input/Output).
   */
  public static final class IOConstants {
    public static final int DRIVER_JOYSTICK_INDEX = 0;

    public static final class DriverButtons {
      public static final int SLOWDOWN1 = 6;
      public static final int SLOWDOWN2 = 5;
    }

    public static final class DriverAxes {
      public static final int LEFT = 1;
      public static final int RIGHT = 5;
    }

    public static final int OPERATOR_JOYSTICK_INDEX = 1;

    public static final class OperatorButtons {
      public static final int INTAKE = 3; // PLACEHOLDER

      public static final int OUTTAKE = 1;

      public static final int TOWER = 2; // PLACEHOLDER

      public static final int CLIMBER_UP = 6;
      public static final int CLIMBER_DOWN = 5;
      public static final int CLIMBER_RETRACT = 11;
      public static final int CLIMBER_EXTEND = 12;
    }
  }

  /**
   * Stores constants related to the DriveBase.
   */
  public static final class DriveConstants {
    public static final double SLOWDOWN_PERCENT1 = 0.75;
    public static final double SLOWDOWN_PERCENT2 = 0.5;

    public static final double ACCELERATION_RATE_LIMIT = 2.1;

    public static final class DriveMotorCANIDs {
      public static final int LEFT_FRONT = 42;
      public static final int LEFT_BACK = 41;
      public static final int RIGHT_FRONT = 44;
      public static final int RIGHT_BACK = 43;
    }

    public static final int CURRENT_LIMIT = 60;
    public static final int TRIGGER_THRESHOLD = 80;
    public static final double TRIGGER_THRESHOLD_TIME = 0.5;
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

    public static final int CURRENT_LIMIT = 40; // PLACEHOLDER
    public static final int TRIGGER_THRESHOLD = 40; // PLACEHOLDER
    public static final int TRIGGER_THRESHOLD_TIME = 2; // PLACEHOLDER
  }

  /**
   * Stores constants related to the Outtake.
   */
  public static final class OuttakeConstants {
    public static final double SPEED = -0.4;

    public static final class OuttakeMotorCANIDs {
      public static final int TOP = 62;
      public static final int BOTTOM = 61;
    }

    public static final int CURRENT_LIMIT = 40;
  }

  /**
   * Stores constants related to the Tower.
   */
  public static final class TowerConstants {
    public static final double SPEED = 0.4; // PLACEHOLDER

    public static final class TowerMotorCANIDs {
      public static final int TOP_LEFT = 51; // PLACEHOLDER
      public static final int BOTTOM_LEFT = 53; // PLACEHOLDER
      public static final int TOP_RIGHT = 52; // PLACEHOLDER
      public static final int BOTTOM_RIGHT = 54; // PLACEHOLDER
    }

    public static final int CURRENT_LIMIT = 25; // PLACEHOLDER
  }

  /**
   * Stores constants related to the Climber.
   */
  public static final class ClimberConstants {
    public static final double SPEED = 1.0;

    public static final class ClimberMotorCANIDs {
      public static final int LEFT = 31;
      public static final int RIGHT = 32;
    }

    public static final int CURRENT_LIMIT = 85;
    public static final int TRIGGER_THRESHOLD = 100;
    public static final double TRIGGER_THRESHOLD_TIME = 0.5;

    public static final class ClimberPositions {
      public static final int RETRACTED = 0;
      public static final int EXTENDED = -170000;
    }

    public static final class ClimberPIDConstants {
      public static final double kP_LEFT = 0.1;
      public static final double kP_RIGHT = 0.1;
      public static final double kD_LEFT = 0.01;
      public static final double kD_RIGHT = 0.01;
      public static final double kI_LEFT = 0;
      public static final double kI_RIGHT = 0;

      public static final int timeoutMs = 0;
      public static final int kSlotIdx = 0;
      public static final int CURVE_STRENGTH = 0;
      public static final int ACCELERATION = 400000;
      public static final int VELOCITY = 400000;
    }
  }

  /**
   * Stores constants related to the Pneumatics.
   */
  public final class PneumaticsConstants {
    public static final double DEFAULT_MIN_PRESSURE = 90.0; // PLACEHOLDER
    public static final double DEFAULT_MAX_PRESSURE = 115.0; // PLACEHOLDER
  }

}
