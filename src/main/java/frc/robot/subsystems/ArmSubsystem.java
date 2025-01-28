package frc.robot.subsystems;

import static com.ctre.phoenix6.signals.ForwardLimitSourceValue.RemoteCANdiS1;
import static com.ctre.phoenix6.signals.NeutralModeValue.Brake;
import static com.ctre.phoenix6.signals.ReverseLimitSourceValue.RemoteCANdiS2;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Constants.ArmConstants.ARM_MAGNETIC_OFFSET;
import static frc.robot.Constants.ArmConstants.ARM_MOTION_MAGIC_CONFIGS;
import static frc.robot.Constants.ArmConstants.ARM_POSITION_TOLERANCE;
import static frc.robot.Constants.ArmConstants.ARM_SLOT_CONFIGS;
import static frc.robot.Constants.ArmConstants.ARM_STATOR_CURRENT_LIMIT;
import static frc.robot.Constants.ArmConstants.ARM_SUPPLY_CURRENT_LIMIT;
import static frc.robot.Constants.ArmConstants.DEVICE_ID_ARM_CANCODER;
import static frc.robot.Constants.ArmConstants.DEVICE_ID_ARM_MOTOR;
import static frc.robot.Constants.ArmConstants.DEVICE_ID_ELEVATOR_CANDI;
import static frc.robot.Constants.ArmConstants.DEVICE_ID_ELEVATOR_MOTOR_FOLLOWER;
import static frc.robot.Constants.ArmConstants.DEVICE_ID_ELEVATOR_MOTOR_LEADER;
import static frc.robot.Constants.ArmConstants.ELEVATOR_BOTTOM_LIMIT;
import static frc.robot.Constants.ArmConstants.ELEVATOR_DEFAULT_HEIGHT;
import static frc.robot.Constants.ArmConstants.ELEVATOR_METERS_PER_ROTATION;
import static frc.robot.Constants.ArmConstants.ELEVATOR_MOTION_MAGIC_CONFIGS;
import static frc.robot.Constants.ArmConstants.ELEVATOR_POSITION_TOLERANCE;
import static frc.robot.Constants.ArmConstants.ELEVATOR_SLOT_CONFIGS;
import static frc.robot.Constants.ArmConstants.ELEVATOR_SUPPLY_CURRENT_LIMIT;
import static frc.robot.Constants.ArmConstants.ELEVATOR_TOP_LIMIT;
import static frc.robot.Constants.ArmConstants.INTAKE_ANGLE;
import static frc.robot.Constants.ArmConstants.LEVEL_1_HEIGHT;
import static frc.robot.Constants.ArmConstants.LEVEL_2_ANGLE;
import static frc.robot.Constants.ArmConstants.LEVEL_2_HEIGHT;
import static frc.robot.Constants.ArmConstants.LEVEL_3_ANGLE;
import static frc.robot.Constants.ArmConstants.LEVEL_3_HEIGHT;
import static frc.robot.Constants.ArmConstants.LEVEL_4_ANGLE;
import static frc.robot.Constants.ArmConstants.LEVEL_4_HEIGHT;
import static frc.robot.Constants.CANIVORE_BUS_NAME;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.CANdi;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

/**
 * The is the Subsytem for the Arm, including the elevator.
 */
public class ArmSubsystem implements Subsystem {

  private final TalonFX elevatorMotorLeader = new TalonFX(DEVICE_ID_ELEVATOR_MOTOR_LEADER, CANIVORE_BUS_NAME);
  private final TalonFX elevatorMotorFollower = new TalonFX(DEVICE_ID_ELEVATOR_MOTOR_FOLLOWER, CANIVORE_BUS_NAME);
  private final CANdi canDiElevator = new CANdi(DEVICE_ID_ELEVATOR_CANDI, CANIVORE_BUS_NAME);

  private final TalonFX armMotor = new TalonFX(DEVICE_ID_ARM_MOTOR, CANIVORE_BUS_NAME);

  private final MotionMagicVoltage elevatorControl = new MotionMagicVoltage(0.0);
  private final MotionMagicVoltage armControl = new MotionMagicVoltage(0.0);
  private final VoltageOut sysIdElevatorControl = new VoltageOut(0.0);
  private final VoltageOut sysIdArmControl = new VoltageOut(0.0);

  private final SysIdRoutine elevatorSysIdRoutine = new SysIdRoutine(
      new SysIdRoutine.Config(
          null,
          null,
          null,
          state -> SignalLogger.writeString("Elevator Motor SysId", state.toString())),
      new SysIdRoutine.Mechanism((voltage) -> {
        elevatorMotorLeader.setControl(sysIdElevatorControl.withOutput(voltage.in(Volts)));
      }, null, this));

  private final SysIdRoutine armSysIdRoutine = new SysIdRoutine(
      new SysIdRoutine.Config(null, null, null, state -> SignalLogger.writeString("Arm Motor SysId", state.toString())),
      new SysIdRoutine.Mechanism((voltage) -> {
        elevatorMotorLeader.setControl(sysIdArmControl.withOutput(voltage.in(Volts)));
      }, null, this));

  private final StatusSignal<Boolean> atTopLimitSignal = canDiElevator.getS1Closed();
  private final StatusSignal<Boolean> atBottomLimitSignal = canDiElevator.getS2Closed();
  private final StatusSignal<Angle> elevatorRotations = elevatorMotorLeader.getPosition();
  private final StatusSignal<Angle> armRotations = armMotor.getPosition();

  /**
   * Creates a new ArmSubsystem.
   */
  public ArmSubsystem() {
    var elevatorTalonConfig = new TalonFXConfiguration();
    elevatorTalonConfig.MotorOutput.withNeutralMode(Brake);
    elevatorTalonConfig.withSlot0(Slot0Configs.from(ELEVATOR_SLOT_CONFIGS));
    elevatorTalonConfig.CurrentLimits.withSupplyCurrentLimit(ELEVATOR_SUPPLY_CURRENT_LIMIT)
        .withSupplyCurrentLimitEnable(true);
    elevatorTalonConfig.withMotionMagic(ELEVATOR_MOTION_MAGIC_CONFIGS);
    elevatorTalonConfig.HardwareLimitSwitch.withForwardLimitRemoteSensorID(canDiElevator.getDeviceID())
        .withForwardLimitSource(RemoteCANdiS1)
        .withReverseLimitRemoteSensorID(canDiElevator.getDeviceID())
        .withReverseLimitSource(RemoteCANdiS2);
    elevatorTalonConfig.SoftwareLimitSwitch.withForwardSoftLimitEnable(true)
        .withForwardSoftLimitThreshold(ELEVATOR_TOP_LIMIT.in(Meters) / ELEVATOR_METERS_PER_ROTATION.in(Meters))
        .withReverseSoftLimitEnable(true)
        .withReverseSoftLimitThreshold(ELEVATOR_BOTTOM_LIMIT.in(Meters) / ELEVATOR_METERS_PER_ROTATION.in(Meters));

    elevatorMotorLeader.getConfigurator().apply(elevatorTalonConfig);
    elevatorMotorFollower.getConfigurator().apply(elevatorTalonConfig);
    elevatorMotorFollower.setControl(new Follower(elevatorMotorLeader.getDeviceID(), false));

    var armCanCoder = new CANcoder(DEVICE_ID_ARM_CANCODER, CANIVORE_BUS_NAME);
    var armCanCoderConfig = new CANcoderConfiguration();
    armCanCoderConfig.MagnetSensor.withMagnetOffset(ARM_MAGNETIC_OFFSET);
    armCanCoder.getConfigurator().apply(armCanCoderConfig);

    var armTalonConfig = new TalonFXConfiguration();
    armTalonConfig.MotorOutput.withNeutralMode(Brake);
    armTalonConfig.withSlot0(Slot0Configs.from(ARM_SLOT_CONFIGS));
    armTalonConfig.CurrentLimits.withSupplyCurrentLimit(ARM_SUPPLY_CURRENT_LIMIT)
        .withSupplyCurrentLimitEnable(true)
        .withStatorCurrentLimit(ARM_STATOR_CURRENT_LIMIT)
        .withStatorCurrentLimitEnable(true);
    armTalonConfig.withMotionMagic(ARM_MOTION_MAGIC_CONFIGS);
    armTalonConfig.Feedback.withFusedCANcoder(armCanCoder);

    armMotor.getConfigurator().apply(armTalonConfig);
  }

  /**
   * Command to run Elevator SysId routine in dynamic mode
   * 
   * @param direction The direction to run the elevator motor for dynamic mode
   * @return The SysId output data for dynamic mode
   */
  public Command sysIdElevatorDynamicCommand(Direction direction) {
    return elevatorSysIdRoutine.dynamic(direction)
        .withName("SysId elevator dynamic " + direction)
        .finallyDo(this::stopElevator);
  }

  /**
   * Command to run Elevator SysId routine in quasistatic mode
   * 
   * @param direction The direction to run the elevator motor for quasistatic mode
   * @return The SysId output data for quasistatic mode
   */
  public Command sysIdElevatorQuasistaticCommand(Direction direction) {
    return elevatorSysIdRoutine.quasistatic(direction)
        .withName("SysId elevator quasi " + direction)
        .finallyDo(this::stopElevator);
  }

  /**
   * Command to run Elevator SysId routine in dynamic mode
   * 
   * @param direction The direction to run the elevator motor for dynamic mode
   * @return The SysId output data for dynamic mode
   */
  public Command sysIdArmDynamicCommand(Direction direction) {
    return armSysIdRoutine.dynamic(direction).withName("SysId arm dynamic " + direction).finallyDo(this::stopArm);
  }

  /**
   * Command to run Elevator SysId routine in quasistatic mode
   * 
   * @param direction The direction to run the elevator motor for quasistatic mode
   * @return The SysId output data for quasistatic mode
   */
  public Command sysIdArmQuasistaticCommand(Direction direction) {
    return armSysIdRoutine.quasistatic(direction).withName("SysId arm quasi " + direction).finallyDo(this::stopArm);
  }

  /**
   * Indicates if the top limit switch is tripped
   * 
   * @return true if the limit switch is tripped, otherwise false
   */
  public boolean isAtTopLimit() {
    return atTopLimitSignal.refresh().getValue();
  }

  /**
   * Indicates if the bottom limit switch is tripped
   * 
   * @return true if the bottom limit switch is tripped, otherwise false
   */
  public boolean isAtBottomLimit() {
    return atBottomLimitSignal.refresh().getValue();
  }

  /**
   * Moves the elevator to a position measured in meters.
   *
   * @param The desired position in meters
   */
  public void moveElevator(Distance position) {
    elevatorMotorLeader
        .setControl(elevatorControl.withPosition(position.in(Meters) * ELEVATOR_METERS_PER_ROTATION.in(Meters)));
  }

  /**
   * Moves the elevator to the height it remains at when inactive
   */
  public void moveElevatorToDefault() {
    moveElevator(ELEVATOR_DEFAULT_HEIGHT);
  }

  /*
   * Moves the elevator to the height required to score on the bottom level
   */
  public void moveElevatorLevel1() {
    moveElevator(LEVEL_1_HEIGHT);
  }

  /*
   * Moves the elevator to the height required to score on the second level
   */
  public void moveElevatorLevel2() {
    moveElevator(LEVEL_2_HEIGHT);
  }

  /*
   * Moves the elevator to the height required to score on the third level
   */
  public void moveElevatorLevel3() {
    moveElevator(LEVEL_3_HEIGHT);
  }

  /*
   * Moves the elevator to the height required to score on the top level
   */
  public void moveElevatorLevel4() {
    moveElevator(LEVEL_4_HEIGHT);
  }

  /**
   * Stops the elevator
   */
  public void stopElevator() {
    elevatorMotorLeader.stopMotor();
  }

  /**
   * Stops the arm
   */
  public void stopArm() {
    armMotor.stopMotor();
  }

  /**
   * Moves the arm to reef level 2
   */
  public void moveArmToLevel2() {
    moveArmToAngle(LEVEL_2_ANGLE);
  }

  /**
   * Moves the arm to reef level 3
   */
  public void moveArmToLevel3() {
    moveArmToAngle(LEVEL_3_ANGLE);
  }

  /**
   * Moves the arm to reef level 4
   */
  public void moveArmToLevel4() {
    moveArmToAngle(LEVEL_4_ANGLE);
  }

  /**
   * Moves the arm to the intake angle
   */
  public void moveArmToIntake() {
    moveArmToAngle(INTAKE_ANGLE);
  }

  /**
   * Moves the arm to the specified angle
   *
   * @param targetAngle angle to move the arm to
   */
  public void moveArmToAngle(Angle targetAngle) {
    armMotor.setControl(armControl.withPosition(targetAngle));
  }

  /**
   * Checks if the elevator is at the target position
   * 
   * @return true if the elevator is at the target position, within a tolerance
   */
  public boolean isElevatorAtPosition() {
    return Math.abs(elevatorControl.Position - elevatorRotations.getValue().in(Rotations)) < ELEVATOR_POSITION_TOLERANCE
        .in(Rotations);
  }

  /**
   * Checks if the arm is at the target position
   * 
   * @return true if the arm is at the target position, within a tolerance
   */
  public boolean isArmAtPosition() {
    return Math.abs(armControl.Position - armRotations.getValue().in(Rotations)) < ARM_POSITION_TOLERANCE.in(Rotations);
  }
}
