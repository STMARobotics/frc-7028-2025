// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static com.ctre.phoenix6.signals.NeutralModeValue.Brake;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Constants.AlgaeConstants.DEVICE_ID_CANCODER;
import static frc.robot.Constants.AlgaeConstants.DEVICE_ID_CANRANGE;
import static frc.robot.Constants.AlgaeConstants.DEVICE_ID_ROLLERMOTOR;
import static frc.robot.Constants.AlgaeConstants.DEVICE_ID_WRISTMOTOR;
import static frc.robot.Constants.AlgaeConstants.EJECT_SPEED;
import static frc.robot.Constants.AlgaeConstants.INTAKE_SPEED;
import static frc.robot.Constants.AlgaeConstants.ROLLER_STATOR_CURRENT_LIMIT;
import static frc.robot.Constants.AlgaeConstants.ROLLER_SUPPLY_CURRENT_LIMIT;
import static frc.robot.Constants.AlgaeConstants.SCORE_SPEED;
import static frc.robot.Constants.AlgaeConstants.WRIST_DOWN_POSITION;
import static frc.robot.Constants.AlgaeConstants.WRIST_ENCODER_OFFSET;
import static frc.robot.Constants.AlgaeConstants.WRIST_MOTION_MAGIC_CONFIGS;
import static frc.robot.Constants.AlgaeConstants.WRIST_PROCESSOR_POSITION;
import static frc.robot.Constants.AlgaeConstants.WRIST_ROTOR_TO_SENSOR_RATIO;
import static frc.robot.Constants.AlgaeConstants.WRIST_SLOT_CONFIGS;
import static frc.robot.Constants.AlgaeConstants.WRIST_STATOR_CURRENT_LIMIT;
import static frc.robot.Constants.AlgaeConstants.WRIST_SUPPLY_CURRENT_LIMIT;
import static frc.robot.Constants.AlgaeConstants.WRIST_TOLERANCE;
import static frc.robot.Constants.AlgaeConstants.WRIST_UP_POSITION;
import static frc.robot.Constants.CANIVORE_BUS_NAME;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

/**
 * Algae subsystem does all jobs related to algae (intake, scoring, etc).
 */
public class AlgaeSubsystem extends SubsystemBase {

  // define motors
  public final TalonFX rollerMotor = new TalonFX(DEVICE_ID_ROLLERMOTOR, CANIVORE_BUS_NAME);
  public final TalonFX wristMotor = new TalonFX(DEVICE_ID_WRISTMOTOR, CANIVORE_BUS_NAME);
  // define how motors are controlled
  private final VelocityTorqueCurrentFOC rollerControl = new VelocityTorqueCurrentFOC(0.0);
  private final MotionMagicVoltage wristControl = new MotionMagicVoltage(0.0).withEnableFOC(true);
  // define cancoder
  public final static CANcoder cancoder = new CANcoder(DEVICE_ID_CANCODER);
  // define canrange
  public final static CANrange canrange = new CANrange(DEVICE_ID_CANRANGE);

  private final TorqueCurrentFOC rollerCharacterization = new TorqueCurrentFOC(0.0);
  private final VoltageOut voltageCharacterization = new VoltageOut(0.0).withEnableFOC(true);

  // SysId routine for rollers - NOTE: the output type is amps, NOT volts (even though it says volts)
  // https://www.chiefdelphi.com/t/sysid-with-ctre-swerve-characterization/452631/8
  private final SysIdRoutine rollerSysIdRoutine = new SysIdRoutine(
      new SysIdRoutine.Config(
          Volts.of(3.0).per(Second),
          Volts.of(25),
          null,
          state -> SignalLogger.writeString("SysIdAlgaeRoller_State", state.toString())),
      new SysIdRoutine.Mechanism(
          amps -> rollerMotor.setControl(rollerCharacterization.withOutput(amps.in(Volts))),
          null,
          this));

  // SysId routine for deploy
  private final SysIdRoutine wristSysIdRoutine = new SysIdRoutine(
      new SysIdRoutine.Config(
          Volts.of(1).per(Second),
          Volts.of(1),
          null,
          state -> SignalLogger.writeString("SysIdAlgaeDeploy_State", state.toString())),
      new SysIdRoutine.Mechanism(
          output -> wristMotor.setControl(voltageCharacterization.withOutput(output)),
          null,
          this));

  // range keeps track of algae distance
  public StatusSignal<Distance> range = canrange.getDistance();

  // angle keeps track of subsystem angle to control the wrist
  public StatusSignal<Angle> wristAngle = cancoder.getPosition();
  public StatusSignal<AngularVelocity> wristVelocity = cancoder.getVelocity();

  public AlgaeSubsystem() {
    // configure wrist encoder
    var deployEncoder = new CANcoder(DEVICE_ID_CANCODER, CANIVORE_BUS_NAME);
    var wristEncoderConfig = new CANcoderConfiguration();
    wristEncoderConfig.MagnetSensor.withMagnetOffset(WRIST_ENCODER_OFFSET)
        .withSensorDirection(SensorDirectionValue.Clockwise_Positive);
    deployEncoder.getConfigurator().apply(wristEncoderConfig);

    // configure wrist motor
    var wristMotorConfig = new TalonFXConfiguration();
    wristMotorConfig.MotorOutput.withNeutralMode(Brake).withInverted(InvertedValue.Clockwise_Positive);
    wristMotorConfig.CurrentLimits.withStatorCurrentLimit(WRIST_STATOR_CURRENT_LIMIT)
        .withStatorCurrentLimitEnable(true)
        .withSupplyCurrentLimit(WRIST_SUPPLY_CURRENT_LIMIT)
        .withSupplyCurrentLimitEnable(true);
    wristMotorConfig.Feedback.withRotorToSensorRatio(WRIST_ROTOR_TO_SENSOR_RATIO).withFusedCANcoder(deployEncoder);
    wristMotorConfig.withSlot0(Slot0Configs.from(WRIST_SLOT_CONFIGS));
    wristMotorConfig.withMotionMagic(WRIST_MOTION_MAGIC_CONFIGS);
    wristMotor.getConfigurator().apply(wristMotorConfig);

    // configure roller motor
    var rollerMotorConfig = new TalonFXConfiguration();
    rollerMotorConfig.MotorOutput.withNeutralMode(Brake).withInverted(InvertedValue.Clockwise_Positive);
    rollerMotorConfig.CurrentLimits.withStatorCurrentLimit(ROLLER_STATOR_CURRENT_LIMIT)
        .withStatorCurrentLimitEnable(true)
        .withSupplyCurrentLimit(ROLLER_SUPPLY_CURRENT_LIMIT)
        .withSupplyCurrentLimitEnable(true);
    rollerMotorConfig.Slot0 = Slot0Configs.from(WRIST_SLOT_CONFIGS);

    // can configs
    CANrangeConfiguration configs = new CANrangeConfiguration();
  }

  /**
   * Creates a command to run the roller dynamic sysid routine
   * 
   * @param direction direction to run the sysid routine
   * @return command to run the sysid routine
   */
  public Command sysIdRollerDynamicCommand(Direction direction) {
    return rollerSysIdRoutine.dynamic(direction).withName("Roller dynam " + direction).finallyDo(this::stop);
  }

  /**
   * Creates a command to run the roller quasistatic sysid routine
   * 
   * @param direction direction to run the sysid routine
   * @return command to run the sysid routine
   */
  public Command sysIdSRollerQuasistaticCommand(Direction direction) {
    return rollerSysIdRoutine.quasistatic(direction).withName("Roller quasi " + direction).finallyDo(this::stop);
  }

  /**
   * Creates a command to run the deploy dynamic sysid routine
   * 
   * @param direction direction to run the sysid routine
   * @return command to run the sysid routine
   */
  public Command sysIdDeployDynamicCommand(Direction direction) {
    return wristSysIdRoutine.dynamic(direction).withName("Deploy dynam " + direction).finallyDo(this::stop);
  }

  /**
   * Creates a command to run the deploy quasistatic sysid routine
   * 
   * @param direction direction to run the sysid routine
   * @return command to run the sysid routine
   */
  public Command sysIdDeployQuasistaticCommand(Direction direction) {
    return wristSysIdRoutine.quasistatic(direction).withName("Deploy quasi " + direction).finallyDo(this::stop);
  }

  public void runRollers(AngularVelocity speed) {
    rollerMotor.setControl(rollerControl.withVelocity(speed));
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
  public void eject() {
    rollerMotor.setControl(rollerControl.withVelocity(EJECT_SPEED));
  }

  /**
   * Stops motors entirely
   */
  public void stop() {
    rollerMotor.stopMotor();
    wristMotor.stopMotor();
  }

  /**
   * Drops the algae to the processor from the rollers. Use {@link #moveToProcessorPosition()} to move the intake to the
   * processor position first.
   */
  public void dropAlgaeToProcess() {
    rollerMotor.setControl(rollerControl.withVelocity(SCORE_SPEED));
  }

  /**
   * Deploy to the position to score in the processor
   */
  public void moveToProcessorPosition() {
    wristMotor.setControl(wristControl.withPosition(WRIST_PROCESSOR_POSITION));
  }

  /**
   * Checks if the wrist is at the target position
   * 
   * @return true if the wrist is at the target position, within a tolerance
   */
  public boolean isWristAtPosition() {
    BaseStatusSignal.refreshAll(wristAngle, wristVelocity);
    var position = BaseStatusSignal.getLatencyCompensatedValue(wristAngle, wristVelocity);
    return Math.abs(wristControl.Position - position.in(Rotations)) < WRIST_TOLERANCE.in(Rotations);
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

  /*
   * Command to get the speed of the algae rollers
   */
  public double getRollerSpeed() {
    return wristVelocity.getValueAsDouble();
  }
}
