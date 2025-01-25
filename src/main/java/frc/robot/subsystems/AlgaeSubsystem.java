// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.AlgaeConstants.ALGAE_SLOT_CONFIGS;
import static frc.robot.Constants.AlgaeConstants.DEVICE_ID_CANCODER;
import static frc.robot.Constants.AlgaeConstants.DEVICE_ID_CANRANGE;
import static frc.robot.Constants.AlgaeConstants.DEVICE_ID_ROLLERMOTOR;
import static frc.robot.Constants.AlgaeConstants.DEVICE_ID_WRISTMOTOR;
import static frc.robot.Constants.AlgaeConstants.INTAKE_SPEED;
import static frc.robot.Constants.AlgaeConstants.OUTTAKE_SPEED;
import static frc.robot.Constants.AlgaeConstants.SCORE_SPEED;
import static frc.robot.Constants.AlgaeConstants.WRIST_DOWN_POSITION;
import static frc.robot.Constants.AlgaeConstants.WRIST_UP_POSITION;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Algae subsystem does all jobs related to algae (intake, scoring, etc).
 */
public class AlgaeSubsystem extends SubsystemBase {

  // define motors
  public final TalonFX rollerMotor = new TalonFX(DEVICE_ID_ROLLERMOTOR);
  public final TalonFX wristMotor = new TalonFX(DEVICE_ID_WRISTMOTOR);
  // define how motors are controlled
  private final VelocityTorqueCurrentFOC rollerControl = new VelocityTorqueCurrentFOC(0.0);
  private final PositionVoltage wristControl = new PositionVoltage(0.0);
  // define cancoder
  public final static CANcoder cancoder = new CANcoder(DEVICE_ID_CANCODER);
  // define canrange
  public final static CANrange canrange = new CANrange(DEVICE_ID_CANRANGE);

  // variables (yet to actually be used anywhere)

  // range keeps track of algae distance
  public StatusSignal<Distance> range = canrange.getDistance();

  // angle keeps track of subsystem angle to control the wrist
  public StatusSignal<Angle> angle = cancoder.getPosition();

  public AlgaeSubsystem() {
    // motor configs
    var rollerMotorConfig = new TalonFXConfiguration();
    var wristMotorConfig = new TalonFXConfiguration();

    rollerMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    wristMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    rollerMotorConfig.Slot0 = Slot0Configs.from(ALGAE_SLOT_CONFIGS);
    wristMotorConfig.Slot0 = Slot0Configs.from(ALGAE_SLOT_CONFIGS);

    // can configs
    CANrangeConfiguration configs = new CANrangeConfiguration();
  }

  /**
   * Activates motor to activate rollers and intake algae
   */
  public void intake() {
    rollerMotor.setControl(rollerControl.withVelocity(INTAKE_SPEED));
  }

  /**
   * Reverses motors to push it out
   */
  public void outtake() {
    rollerMotor.setControl(rollerControl.withVelocity(OUTTAKE_SPEED));
  }

  /**
   * Stops motors entirely
   */
  public void stop() {
    rollerMotor.stopMotor();
    wristMotor.stopMotor();
  }

  /**
   * Scores intake, does same thing as outtake but at a different speed
   */
  public void score() {
    rollerMotor.setControl(rollerControl.withVelocity(SCORE_SPEED));
  }

  /**
   * Moves intake to the floor position in order to pick up
   */
  public void moveIntakeDown() {
    wristMotor.setControl(wristControl.withPosition(WRIST_DOWN_POSITION));
  }

  /**
   * Moves intake to the upwards position for holding/scoring
   */
  public void moveIntakeUp() {
    wristMotor.setControl(wristControl.withPosition(WRIST_UP_POSITION));
  }
}
