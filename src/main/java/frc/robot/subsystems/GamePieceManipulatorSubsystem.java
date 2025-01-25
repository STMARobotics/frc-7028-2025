// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Constants.CANIVORE_BUS_NAME;
import static frc.robot.Constants.GamePieceManipulatorConstants.DEVICE_ID_MANIPULATOR_MOTOR;
import static frc.robot.Constants.GamePieceManipulatorConstants.EJECT_ALGAE_VELOCITY;
import static frc.robot.Constants.GamePieceManipulatorConstants.EJECT_SPEED;
import static frc.robot.Constants.GamePieceManipulatorConstants.HOLD_SLOT_CONFIGS;
import static frc.robot.Constants.GamePieceManipulatorConstants.INTAKE_ALGAE_VELOCITY;
import static frc.robot.Constants.GamePieceManipulatorConstants.INTAKE_SPEED;
import static frc.robot.Constants.GamePieceManipulatorConstants.MANIPULATION_SLOT_CONFIGS;
import static frc.robot.Constants.GamePieceManipulatorConstants.SCORE_ALGAE_VELOCITY;
import static frc.robot.Constants.GamePieceManipulatorConstants.SCORE_SPEED;
import static frc.robot.Constants.GamePieceManipulatorConstants.STATOR_CURRENT_LIMIT;
import static frc.robot.Constants.IndexerConstants.SUPPLY_CURRENT_LIMIT;
import static frc.robot.Constants.IndexerConstants.TORQUE_CURRENT_LIMIT;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

/**
 * The is the Subsytem for the Game Pieace Manipulator.
 */
public class GamePieceManipulatorSubsystem extends SubsystemBase {

  // define motors
  private final TalonFX wheelMotor = new TalonFX(DEVICE_ID_MANIPULATOR_MOTOR, CANIVORE_BUS_NAME);

  private final StatusSignal<Angle> positionSignal = wheelMotor.getPosition();
  private final StatusSignal<AngularVelocity> velocitySignal = wheelMotor.getVelocity();

  private final TorqueCurrentFOC wheelCharacterization = new TorqueCurrentFOC(0.0);

  // SysId routine for rollers - NOTE: the output type is amps, NOT volts (even though it says volts)
  // https://www.chiefdelphi.com/t/sysid-with-ctre-swerve-characterization/452631/8
  private final SysIdRoutine manipulatorSysIdRoutine = new SysIdRoutine(
      new SysIdRoutine.Config(
          Volts.of(3.0).per(Second),
          Volts.of(25),
          null,
          state -> SignalLogger.writeString("SysIdGamePieceManipulator_State", state.toString())),
      new SysIdRoutine.Mechanism(
          amps -> wheelMotor.setControl(wheelCharacterization.withOutput(amps.in(Volts))),
          null,
          this));

  // define how motors are controlled
  private final VelocityTorqueCurrentFOC wheelControl = new VelocityTorqueCurrentFOC(0).withSlot(0);
  private final PositionVoltage holdControl = new PositionVoltage(0.0).withSlot(1);

  private StatusSignal<AngularVelocity> manipulatorSpeed;

  /** Creates a new Subsytem for the Game Pieace Manipulator. */
  public GamePieceManipulatorSubsystem() {

    manipulatorSpeed = wheelMotor.getVelocity();
  }

  /**
   * Runs the manipulator wheels at any specific speed
   * 
   * @param speed to run the belt in radians per second
   */
  public void runManipulatorWheels(AngularVelocity speed) {
    wheelMotor.setControl(wheelControl.withVelocity(speed));
    var motorConfig = new TalonFXConfiguration();
    motorConfig.withSlot0(Slot0Configs.from(MANIPULATION_SLOT_CONFIGS)).withSlot1(Slot1Configs.from(HOLD_SLOT_CONFIGS));
    motorConfig.MotorOutput.withNeutralMode(NeutralModeValue.Brake);
    motorConfig.TorqueCurrent.withPeakForwardTorqueCurrent(TORQUE_CURRENT_LIMIT)
        .withPeakReverseTorqueCurrent(TORQUE_CURRENT_LIMIT.unaryMinus());
    motorConfig.CurrentLimits.withStatorCurrentLimit(STATOR_CURRENT_LIMIT)
        .withStatorCurrentLimitEnable(true)
        .withSupplyCurrentLimit(SUPPLY_CURRENT_LIMIT)
        .withSupplyCurrentLimitEnable(true);
    wheelMotor.getConfigurator().apply(motorConfig);
  }

  /**
   * Creates a command to run the manipulator dynamic sysid routine
   * 
   * @param direction direction to run the sysid routine
   * @return command to run the sysid routine
   */
  public Command sysIdManipulatorDynamicCommand(Direction direction) {
    return manipulatorSysIdRoutine.dynamic(direction)
        .withName("Game Piece Manipulator dynam " + direction)
        .finallyDo(this::stop);
  }

  /**
   * Creates a command to run the manipulator quasistatic sysid routine
   * 
   * @param direction direction to run the sysid routine
   * @return command to run the sysid routine
   */
  public Command sysIdSManipulatorQuasistaticCommand(Direction direction) {
    return manipulatorSysIdRoutine.quasistatic(direction)
        .withName("Game Piece Manipulator quasi " + direction)
        .finallyDo(this::stop);
  }

  /**
   * Allow the wheels to move forward so they can grab the coral from the inside.
   */
  public void intakeCoral() {
    wheelMotor.setControl(wheelControl.withVelocity(INTAKE_SPEED));
  }

  /**
   * Allow the wheel to go backward do the the coral can go on the reef.
   */
  public void ejectCoral() {
    wheelMotor.setControl(wheelControl.withVelocity(EJECT_SPEED));
  }

  /**
   * score spped to put the coral onto the reef.
   */
  public void scoreCoral() {
    wheelMotor.setControl(wheelControl.withVelocity(SCORE_SPEED));
  }

  /**
   * Intake algae
   */
  public void intakeAlgae() {
    wheelMotor.setControl(wheelControl.withVelocity(INTAKE_ALGAE_VELOCITY));
  }

  /**
   * Eject algae
   */
  public void ejectAlgae() {
    wheelMotor.setControl(wheelControl.withVelocity(EJECT_ALGAE_VELOCITY));
  }

  /**
   * Score algae in the net
   */
  public void scoreAlgae() {
    wheelMotor.setControl(wheelControl.withVelocity(SCORE_ALGAE_VELOCITY));
  }

  /**
   * Actively holds the wheels in the current position to hold a game piece in place
   */
  public void activeHoldGamePiece() {
    BaseStatusSignal.refreshAll(positionSignal, velocitySignal);
    var position = BaseStatusSignal.getLatencyCompensatedValue(positionSignal, velocitySignal);
    wheelMotor.setControl(holdControl.withPosition(position.in(Rotations)));
  }

  /**
   * This stops the wheels.
   */
  public void stop() {
    wheelMotor.stopMotor();
  }

  public double getManipulatorSpeed() {
    return manipulatorSpeed.getValueAsDouble();
  }
}