// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final double SIMPLE_AUTON_SPEED = 0.2; // PLACEHOLDER
  public static final double SIMPLE_AUTON_RUNTIME = 5.0; // PLACEHOLDER
  public static final double MEDIUM_AUTON_OUTAKE_RUNTIME = 5.0; // PLACEHOLDER

  /**
   * Stores constants related to driver controls, SmartDashboard and other IO (Input/Output).
   */
  public final class IOConstants {
    public static final int PRIMARY_DRIVER_JOYSTICK_PORT = 0;
    public static final int SECONDARY_DRIVER_JOYSTICK_PORT = 1;

    public static final int FRONT_INTAKE_BUTTON_NUMBER = 0; // PLACEHOLDER
    public static final int BACK_INTAKE_BUTTON_NUMBER = 1; // PLACEHOLDER
    
    public static final int OUTTAKE_BUTTON_NUMBER = 1; // PLACEHOLDER
    public static final int TOWER_BUTTON_NUMBER = 1; // PLACEHOLDER
  }

  /**
   * Stores constants related to the Intake.
   */
  public final class IntakeConstants {
    public static final double INTAKE_SPEED = 1.0; // PLACEHOLDER

    public static final int FRONT_INTAKE_MOTOR_ID = 0; // PLACEHOLDER
    public static final int FRONT_INTAKE_FORWARD_CHANNEL = 0; // PLACEHOLDER
    public static final int FRONT_INTAKE_REVERSE_CHANNEL = 1; // PLACEHOLDER

    public static final int BACK_INTAKE_MOTOR_ID = 1; // PLACEHOLDER
    public static final int BACK_INTAKE_FORWARD_CHANNEL = 0; // PLACEHOLDER
    public static final int BACK_INTAKE_REVERSE_CHANNEL = 1; // PLACEHOLDER
  }

  /**
   * Stores constants related to the Outtake.
   */
  public final class OuttakeConstants {
    public static final double OUTTAKE_SPEED = 0.5; // PLACEHOLDER
    public static final int OUTTAKE_MOTOR_ID1 = 1; // PLACEHOLDER
    public static final int OUTTAKE_MOTOR_ID2 = 2; // PLACEHOLDER
  }

  /**
   * Stores constants related to the Tower.
   */
  public final class TowerConstants {
    public static final double TOWER_SPEED = 1.0; // PLACEHOLDER
    public static final int TOWER_MOTOR_ID = 1; // PLACEHOLDER
  }


}
