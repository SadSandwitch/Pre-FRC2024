// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class RobotConstants{
    //Wheel circumfirence
    public static final double WHEEL_DIAMETER = Units.inchesToMeters(6);
    public static final double WHEEL_CIRCUM = 2*Math.PI*WHEEL_DIAMETER;
    public static final double TRACK_WIDTH = Units.inchesToMeters(21.75);
  }

  public static class DriveConstants {
    //sticks
    public static final int XBOX_PORT_ID = 0;
    
    //motor controller IDs
    public static final int RIGHT_LEAD_ID = 0;
    public static final int RIGHT_FOLLOW_ID = 1;
    public static final int LEFT_LEAD_ID = 2;
    public static final int LEFT_FOLLOW_ID = 3;

    //inversion
    public static final boolean LEFT_DRIVE_INVERT = false;
    public static final boolean RIGHT_DRIVE_INVERT = true;
  }

  public static class SensorConstants {
    //Encoders
    public static final int LEFT_ENCODER_ID = 1;
    public static final int LEFT_ENCODER_BUSID = 2;
    public static final int RIGHT_ENCODER_ID = 3;
    public static final int RIGHT_ENCODER_BUSID = 4;
    //Encoder Units
    public static final double ENCODER_COUNTS_PER_ROT = 1025;
  }
}
