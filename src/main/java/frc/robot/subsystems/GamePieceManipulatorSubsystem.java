// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems;

import static frc.robot.Constants.GamePieceManipulatorConstants.DEVICE_ID_MANIPULATORMOTOR;
import static frc.robot.Constants.GamePieceManipulatorConstants.INTAKE_SPEED;
import static frc.robot.Constants.GamePieceManipulatorConstants.OUTTAKE_SPEED;
import static frc.robot.Constants.GamePieceManipulatorConstants.SCORE_SPEED;

import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * The is the Subsytem for the Game Pieace Manipulator.
 */
public class GamePieceManipulatorSubsystem extends SubsystemBase {

  // define motors
  private final TalonFX manipulatorMotor = new TalonFX(DEVICE_ID_MANIPULATORMOTOR);

  // define how motors are controlled
  private final VelocityTorqueCurrentFOC wheelControl = new VelocityTorqueCurrentFOC(0);

  /** Creates a new Subsytem for the Game Pieace Manipulator. */
  public GamePieceManipulatorSubsystem() {
  }

  /**
   * Allow the wheels to move forward so they can grab the coral from the inside.
   */
  public void intakeCoral() {
    manipulatorMotor.setControl(wheelControl.withVelocity(INTAKE_SPEED));
  }

  /**
   * Allow the wheel to go backward do the the coral can go on the arm.
   */
  public void outtakeCoral() {
    manipulatorMotor.setControl(wheelControl.withVelocity(OUTTAKE_SPEED));
  }

  /**
   * score spped to put the coral onto the reef.
   */
  public void scoreCoral() {
    manipulatorMotor.setControl(wheelControl.withVelocity(SCORE_SPEED));
  }

  /**
   * This stops the wheels.
   */
  public void stop() {
    manipulatorMotor.stopMotor();
  }
}