package frc.robot.subsystems;

import static com.ctre.phoenix6.signals.ForwardLimitSourceValue.RemoteCANdiS1;
import static com.ctre.phoenix6.signals.InvertedValue.Clockwise_Positive;
import static com.ctre.phoenix6.signals.InvertedValue.CounterClockwise_Positive;
import static com.ctre.phoenix6.signals.NeutralModeValue.Brake;
import static com.ctre.phoenix6.signals.ReverseLimitSourceValue.RemoteCANdiS2;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Rotation;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Constants.ArmConstants.ARM_MAGNETIC_OFFSET;
import static frc.robot.Constants.ArmConstants.ARM_MOTION_MAGIC_CONFIGS;
import static frc.robot.Constants.ArmConstants.ARM_PIVOT_LENGTH;
import static frc.robot.Constants.ArmConstants.ARM_POSITION_TOLERANCE;
import static frc.robot.Constants.ArmConstants.ARM_ROTOR_TO_SENSOR_RATIO;
import static frc.robot.Constants.ArmConstants.ARM_SLOT_CONFIGS;
import static frc.robot.Constants.ArmConstants.ARM_STATOR_CURRENT_LIMIT;
import static frc.robot.Constants.ArmConstants.ARM_SUPPLY_CURRENT_LIMIT;
import static frc.robot.Constants.ArmConstants.DEVICE_ID_ARM_CANDI;
import static frc.robot.Constants.ArmConstants.DEVICE_ID_ARM_MOTOR;
import static frc.robot.Constants.ArmConstants.DEVICE_ID_ELEVATOR_CANDI;
import static frc.robot.Constants.ArmConstants.DEVICE_ID_ELEVATOR_MOTOR_FOLLOWER;
import static frc.robot.Constants.ArmConstants.DEVICE_ID_ELEVATOR_MOTOR_LEADER;
import static frc.robot.Constants.ArmConstants.ELEVATOR_BASE_HEIGHT;
import static frc.robot.Constants.ArmConstants.ELEVATOR_BOTTOM_LIMIT;
import static frc.robot.Constants.ArmConstants.ELEVATOR_DISTANCE_PER_ROTATION;
import static frc.robot.Constants.ArmConstants.ELEVATOR_INTAKE_POSITION;
import static frc.robot.Constants.ArmConstants.ELEVATOR_MOTION_MAGIC_CONFIGS;
import static frc.robot.Constants.ArmConstants.ELEVATOR_PARK_HEIGHT;
import static frc.robot.Constants.ArmConstants.ELEVATOR_PARK_TOLERANCE;
import static frc.robot.Constants.ArmConstants.ELEVATOR_POSITION_TOLERANCE;
import static frc.robot.Constants.ArmConstants.ELEVATOR_SLOT_CONFIGS;
import static frc.robot.Constants.ArmConstants.ELEVATOR_STATOR_CURRENT_LIMIT;
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

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANdiConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANdi;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.S1CloseStateValue;
import com.ctre.phoenix6.signals.S2CloseStateValue;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Robot;

/**
 * The is the Subsytem for the Arm, including the elevator.
 */
public class ArmSubsystem extends SubsystemBase {

  private final TalonFX elevatorMotorLeader = new TalonFX(DEVICE_ID_ELEVATOR_MOTOR_LEADER, CANIVORE_BUS_NAME);
  private final TalonFX elevatorMotorFollower = new TalonFX(DEVICE_ID_ELEVATOR_MOTOR_FOLLOWER, CANIVORE_BUS_NAME);
  private final CANdi elevatorCanDi = new CANdi(DEVICE_ID_ELEVATOR_CANDI, CANIVORE_BUS_NAME);

  private final TalonFX armMotor = new TalonFX(DEVICE_ID_ARM_MOTOR, CANIVORE_BUS_NAME);
  private final CANdi armCanDi = new CANdi(DEVICE_ID_ARM_CANDI, CANIVORE_BUS_NAME);

  private final MotionMagicVoltage elevatorControl = new MotionMagicVoltage(0.0).withEnableFOC(true);
  private final MotionMagicVoltage armControl = new MotionMagicVoltage(0.0).withEnableFOC(true);
  private final VoltageOut sysIdElevatorControl = new VoltageOut(0.0).withEnableFOC(true);
  private final VoltageOut sysIdArmControl = new VoltageOut(0.0).withEnableFOC(true);

  private final SysIdRoutine elevatorSysIdRoutine = new SysIdRoutine(
      new SysIdRoutine.Config(
          Volts.per(Second).of(.25),
          Volts.of(1),
          Seconds.of(10),
          state -> SignalLogger.writeString("Elevator Motor SysId", state.toString())),
      new SysIdRoutine.Mechanism((voltage) -> {
        elevatorMotorLeader.setControl(sysIdElevatorControl.withOutput(voltage.in(Volts)));
      }, null, this));

  private final SysIdRoutine armSysIdRoutine = new SysIdRoutine(
      new SysIdRoutine.Config(
          Volts.per(Second).of(0.25),
          Volts.of(2.0),
          null,
          state -> SignalLogger.writeString("Arm Motor SysId", state.toString())),
      new SysIdRoutine.Mechanism((voltage) -> {
        armMotor.setControl(sysIdArmControl.withOutput(voltage.in(Volts)));
      }, null, this));

  private final StatusSignal<Boolean> atTopLimitSignal = elevatorCanDi.getS1Closed();
  private final StatusSignal<Boolean> atBottomLimitSignal = elevatorCanDi.getS2Closed();
  private final StatusSignal<Angle> elevatorPositionSignal = elevatorMotorLeader.getPosition();
  private final StatusSignal<Angle> armPositionSignal = armMotor.getPosition();
  private final StatusSignal<AngularVelocity> armVelocitySignal = armMotor.getVelocity();
  private final StatusSignal<AngularVelocity> elevatorVelocitySignal = elevatorMotorLeader.getVelocity();

  // Mechanism is on a 2d plane, so its the elevator viewed from the back (pop-tart side) of the robot
  // The width of the mechanism pallete is big enough to fit the mechanism in all configurations, pus a little margin
  private final Mechanism2d armMechanism = new Mechanism2d(
      ARM_PIVOT_LENGTH.in(Meters) * 2.2,
      ELEVATOR_BASE_HEIGHT.times(2).plus(ARM_PIVOT_LENGTH).in(Meters));
  // Arm root in the center of the pallete
  private final MechanismRoot2d armRoot = armMechanism.getRoot("Arm", ARM_PIVOT_LENGTH.in(Meters), 0.0);
  // Base of the elevator, facing upward (90-degrees relativve to the mechanism)
  private final MechanismLigament2d elevatorBaseLigament = armRoot.append(
      new MechanismLigament2d("elevatorBase", ELEVATOR_BASE_HEIGHT.in(Meters), 90, 20, new Color8Bit(Color.kBlue)));
  // First stage of the elevator, facing upward (0-degrees relative to the first stage)
  private final MechanismLigament2d elevatorStageLigament = elevatorBaseLigament
      .append(new MechanismLigament2d("elevatorStage", 0, 0, 10, new Color8Bit(Color.kYellow)));
  // Arm, facing outward by default (90-degrees relative to the first stage)
  private final MechanismLigament2d armLigament = elevatorStageLigament
      .append(new MechanismLigament2d("arm", ARM_PIVOT_LENGTH.in(Meters), 90, 5, new Color8Bit(Color.kOrange)));

  /**
   * Creates a new ArmSubsystem.
   */
  public ArmSubsystem() {
    var elevatorCanDiConfiguration = new CANdiConfiguration();
    elevatorCanDiConfiguration.DigitalInputs.withS1CloseState(S1CloseStateValue.CloseWhenLow)
        .withS2CloseState(S2CloseStateValue.CloseWhenLow);
    elevatorCanDi.getConfigurator().apply(elevatorCanDiConfiguration);

    var elevatorTalonConfig = new TalonFXConfiguration();
    elevatorTalonConfig.MotorOutput.withNeutralMode(Brake).withInverted(CounterClockwise_Positive);
    elevatorTalonConfig.withSlot0(Slot0Configs.from(ELEVATOR_SLOT_CONFIGS));
    elevatorTalonConfig.CurrentLimits.withSupplyCurrentLimit(ELEVATOR_SUPPLY_CURRENT_LIMIT)
        .withSupplyCurrentLimitEnable(true)
        .withStatorCurrentLimit(ELEVATOR_STATOR_CURRENT_LIMIT)
        .withStatorCurrentLimitEnable(true);
    elevatorTalonConfig.withMotionMagic(ELEVATOR_MOTION_MAGIC_CONFIGS);
    elevatorTalonConfig.HardwareLimitSwitch.withForwardLimitRemoteSensorID(elevatorCanDi.getDeviceID())
        .withForwardLimitSource(RemoteCANdiS1)
        .withReverseLimitRemoteSensorID(elevatorCanDi.getDeviceID())
        .withReverseLimitSource(RemoteCANdiS2)
        .withReverseLimitAutosetPositionEnable(true)
        .withReverseLimitAutosetPositionValue(Rotations.of(0));
    elevatorTalonConfig.SoftwareLimitSwitch.withForwardSoftLimitEnable(true)
        .withForwardSoftLimitThreshold(elevatorDistanceToRotations(ELEVATOR_TOP_LIMIT))
        .withReverseSoftLimitEnable(true)
        .withReverseSoftLimitThreshold(elevatorDistanceToRotations(ELEVATOR_BOTTOM_LIMIT));

    elevatorMotorLeader.getConfigurator().apply(elevatorTalonConfig);
    elevatorMotorFollower.getConfigurator().apply(elevatorTalonConfig);
    elevatorMotorFollower.setControl(new Follower(elevatorMotorLeader.getDeviceID(), false));

    CANdiConfiguration armCanDiConfig = new CANdiConfiguration();
    armCanDiConfig.PWM1.withAbsoluteSensorOffset(ARM_MAGNETIC_OFFSET)
        .withAbsoluteSensorDiscontinuityPoint(0.5)
        .withSensorDirection(true);
    armCanDi.getConfigurator().apply(armCanDiConfig);

    var armTalonConfig = new TalonFXConfiguration();
    armTalonConfig.MotorOutput.withNeutralMode(Brake).withInverted(Clockwise_Positive);
    armTalonConfig.Voltage.withPeakForwardVoltage(2).withPeakReverseVoltage(-2);
    armTalonConfig.withSlot0(Slot0Configs.from(ARM_SLOT_CONFIGS));
    armTalonConfig.CurrentLimits.withSupplyCurrentLimit(ARM_SUPPLY_CURRENT_LIMIT)
        .withSupplyCurrentLimitEnable(true)
        .withStatorCurrentLimit(ARM_STATOR_CURRENT_LIMIT)
        .withStatorCurrentLimitEnable(true);
    armTalonConfig.withMotionMagic(ARM_MOTION_MAGIC_CONFIGS);
    armTalonConfig.Feedback.withRotorToSensorRatio(ARM_ROTOR_TO_SENSOR_RATIO).withFusedCANdiPwm1(armCanDi);

    armMotor.getConfigurator().apply(armTalonConfig);
    SmartDashboard.putData("Arm", armMechanism);
  }

  @Override
  public void periodic() {
    if (Robot.isReal()) {
      // Update the Mechanism2d
      elevatorStageLigament.setLength(getElevatorMeters());
      // 0 is straight down for the ligament, but 0 is straight out to the right for the real arm
      armLigament.setAngle(getArmAngle().in(Degrees) - 90);
    }
  }

  @Override
  public void simulationPeriodic() {
    // For sim, just say the elevator and arm are instantly where they're supposed to be
    elevatorStageLigament.setLength(elevatorAngleToMeters(elevatorControl.getPositionMeasure()));
    armLigament.setAngle(armControl.getPositionMeasure().in(Degrees) - 90);
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
  public void moveElevatorToPosition(Distance position) {
    elevatorMotorLeader.setControl(elevatorControl.withPosition(elevatorDistanceToRotations(position)));
  }

  /**
   * Moves the elevator to the height it remains at when inactive
   */
  public void parkElevator() {
    moveElevatorToPosition(ELEVATOR_PARK_HEIGHT);
  }

  public void moveElevatorToIntakePosition() {
    moveElevatorToPosition(ELEVATOR_INTAKE_POSITION);
  }

  /*
   * Moves the elevator to the height required to score on the bottom level
   */
  public void moveElevatorLevel1() {
    moveElevatorToPosition(LEVEL_1_HEIGHT);
  }

  /*
   * Moves the elevator to the height required to score on the second level
   */
  public void moveElevatorLevel2() {
    moveElevatorToPosition(LEVEL_2_HEIGHT);
  }

  /*
   * Moves the elevator to the height required to score on the third level
   */
  public void moveElevatorLevel3() {
    moveElevatorToPosition(LEVEL_3_HEIGHT);
  }

  /*
   * Moves the elevator to the height required to score on the top level
   */
  public void moveElevatorLevel4() {
    moveElevatorToPosition(LEVEL_4_HEIGHT);
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
    return Math.abs(
        getElevatorMeters()
            - elevatorAngleToMeters(elevatorControl.getPositionMeasure())) <= ELEVATOR_POSITION_TOLERANCE.in(Meters);
  }

  /**
   * Returns true if the elevator is below park position, within a tolerance.
   * 
   * @return true if elevator is parked
   */
  public boolean isElevatorParked() {
    return getElevatorMeters() < (ELEVATOR_PARK_HEIGHT.in(Meters) + ELEVATOR_PARK_TOLERANCE.in(Meters));
  }

  /**
   * Checks if the arm is at the target angle
   * 
   * @return true if the arm is at the target angle, within a tolerance
   */
  public boolean isArmAtAngle() {
    return getArmAngle().abs(Rotations) <= ARM_POSITION_TOLERANCE.in(Rotations);
  }

  private Measure<AngleUnit> getArmAngle() {
    BaseStatusSignal.refreshAll(armPositionSignal, armVelocitySignal);
    var armPosition = BaseStatusSignal.getLatencyCompensatedValue(armPositionSignal, armVelocitySignal);
    return armPosition;
  }

  private double getElevatorMeters() {
    BaseStatusSignal.refreshAll(elevatorPositionSignal, elevatorVelocitySignal);
    var elevatorPosition = BaseStatusSignal.getLatencyCompensatedValue(elevatorPositionSignal, elevatorVelocitySignal);
    return elevatorAngleToMeters(elevatorPosition);
  }

  private double elevatorAngleToMeters(Measure<AngleUnit> angle) {
    return angle.in(Rotations) * ELEVATOR_DISTANCE_PER_ROTATION.in(Meters.per(Rotation));
  }

  private double elevatorDistanceToRotations(Distance height) {
    return height.in(Meters) / ELEVATOR_DISTANCE_PER_ROTATION.in(Meters.per(Rotation));
  }

}
