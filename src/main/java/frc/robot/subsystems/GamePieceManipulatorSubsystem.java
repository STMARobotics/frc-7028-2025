// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems;

import static frc.robot.Constants.GamePieceManipulatorConstants.DEVICE_ID_MANIPULATORMOTOR;
import static frc.robot.Constants.GamePieceManipulatorConstants.INTAKESPEED;
import static frc.robot.Constants.GamePieceManipulatorConstants.OUTTAKESPEED;
import static frc.robot.Constants.GamePieceManipulatorConstants.SCORESPEED;

import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * The is the Subsytem for the Game Pieace Manipulator.
 */
public class GamePieceManipulatorSubsystem extends SubsystemBase {
  /** Creates a new Subsytem for the Game Pieace Manipulator. */

  // define motors
  private TalonFX manipulatorMotor = new TalonFX(DEVICE_ID_MANIPULATORMOTOR);

  // define how motors are controlled
  private final VelocityTorqueCurrentFOC wheelControl = new VelocityTorqueCurrentFOC(0);

  public GamePieceManipulatorSubsystem() {
  }

  /**
   * intakeCoral
   * Allow the wheels to move forward so they can grab the coral from the inside.
   */
  public void intakeCoral() {
    manipulatorMotor.setControl(wheelControl.withVelocity(INTAKESPEED));
  }

  /**
   * outtakeCoral
   * Allow the wheel to go backward do the the coral can go on the arm.
   */
  public void outtakeCoral() {
    manipulatorMotor.setControl(wheelControl.withVelocity(OUTTAKESPEED));
  }

  public void scoreCoral() {
    manipulatorMotor.setControl(wheelControl.withVelocity(SCORESPEED));
  }

  /**
   * This stops the wheels.
   */
  public void stop() {
    manipulatorMotor.stopMotor();
  }
}