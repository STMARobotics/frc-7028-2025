// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.AlgaeConstants.DEVICE_ID_ROLLERMOTOR;

import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * Algae subsystem does all jobs related to algae (intake, scoring, etc).
 */
public class AlgaeSubsystem extends SubsystemBase {

  // define motors
  public final TalonFX rollerMotor = new TalonFX(DEVICE_ID_ROLLERMOTOR);
  public final TalonFX wristMotor = new TalonFX(DEVICE_ID_ROLLERMOTOR);
  // define how motors are controlled
  private final VelocityTorqueCurrentFOC rollerControl = new VelocityTorqueCurrentFOC(0.0);
  private final VelocityTorqueCurrentFOC wristControl = new VelocityTorqueCurrentFOC(0.0);

  public AlgaeSubsystem() {
  }

  /**
   * Activates motor to activate rollers and intake algae
   */
  public void intake() {
    rollerMotor.setControl(rollerControl.withVelocity(Constants.AlgaeConstants.intakeSpeed));
  }

  /**
   * Reverses motors to push it out
   */
  public void outtake() {
    rollerMotor.setControl(rollerControl.withVelocity(Constants.AlgaeConstants.outtakeSpeed));
  }

  /**
   * Stops motors entirely
   */
  public void stop() {
    rollerMotor.stopMotor();
  }

  /**
   * Scores intake, does same thing as outtake but at a different speed
   */
  public void score() {
    rollerMotor.setControl(rollerControl.withVelocity(Constants.AlgaeConstants.scoreSpeed));
  }

  /**
   * Moves intake to the floor position in order to pick up
   */
  public void moveIntakeDown() {
    wristMotor.setControl(wristControl.withVelocity(Constants.AlgaeConstants.wristDownSpeed));
  }

  /**
   * Moves intake to the upwards position for holding/scoring
   */
  public void moveIntakeUp() {
    wristMotor.setControl(wristControl.withVelocity(Constants.AlgaeConstants.wristUpSpeed));
  }
}
