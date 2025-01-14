// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.units.measure.AngularVelocity;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class AlgaeSubsystemConstants {
    public static final int DEVICE_ID_ROLLERMOTOR = 1; //this number probably isn't right, fix it later

    public static final AngularVelocity intakeSpeed = RadiansPerSecond.of(5); //5 is probably a wonky number, change it later
    public static final AngularVelocity outtakeSpeed = RadiansPerSecond.of(-5);
  }

}
