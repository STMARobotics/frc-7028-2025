package frc.robot.subsystems;

import static com.ctre.phoenix6.signals.ForwardLimitSourceValue.RemoteCANdiS1;
import static com.ctre.phoenix6.signals.NeutralModeValue.Brake;
import static com.ctre.phoenix6.signals.ReverseLimitSourceValue.RemoteCANdiS2;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Constants.ArmConstants.BOTTOM_LIMIT;
import static frc.robot.Constants.ArmConstants.DEVICE_ID_CANDI;
import static frc.robot.Constants.ArmConstants.DEVICE_ID_CANDI_ARM;
import static frc.robot.Constants.ArmConstants.DEVICE_ID_CANDI_ELEVATOR;
import static frc.robot.Constants.ArmConstants.DEVICE_ID_ELEVATOR_MOTOR_FOLLOWER;
import static frc.robot.Constants.ArmConstants.DEVICE_ID_ELEVATOR_MOTOR_LEADER;
import static frc.robot.Constants.ArmConstants.HOLDING_ANGLE;
import static frc.robot.Constants.ArmConstants.INTAKE_ANGLE;
import static frc.robot.Constants.ArmConstants.LEVEL_1_HEIGHT;
import static frc.robot.Constants.ArmConstants.LEVEL_2_ANGLE;
import static frc.robot.Constants.ArmConstants.LEVEL_2_HEIGHT;
import static frc.robot.Constants.ArmConstants.LEVEL_3_ANGLE;
import static frc.robot.Constants.ArmConstants.LEVEL_3_HEIGHT;
import static frc.robot.Constants.ArmConstants.LEVEL_4_ANGLE;
import static frc.robot.Constants.ArmConstants.LEVEL_4_HEIGHT;
import static frc.robot.Constants.ArmConstants.METERS_PER_REVOLUTION;
import static frc.robot.Constants.ArmConstants.MOTION_MAGIC_CONFIGS;
import static frc.robot.Constants.ArmConstants.SLOT_CONFIGS;
import static frc.robot.Constants.ArmConstants.SUPPLY_CURRENT_LIMIT;
import static frc.robot.Constants.ArmConstants.TOP_LIMIT;
import static frc.robot.Constants.CANIVORE_BUS_NAME;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANdi;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
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

  private final TalonFX elevatorMotorLeader = new TalonFX(DEVICE_ID_ELEVATOR_MOTOR_LEADER);
  private final TalonFX elevatorMotorFollower = new TalonFX(DEVICE_ID_ELEVATOR_MOTOR_FOLLOWER);

  private final CANdi canDiElevator = new CANdi(DEVICE_ID_CANDI_ELEVATOR, CANIVORE_BUS_NAME);
  private final CANdi canDiArm = new CANdi(DEVICE_ID_CANDI_ARM);

  private final MotionMagicVoltage elevatorControl = new MotionMagicVoltage(0.0);
  private final VoltageOut sysIdElevatorControl = new VoltageOut(0.0);

  private final SysIdRoutine elevatorSysIdRoutine = new SysIdRoutine(
      new SysIdRoutine.Config(
          null,
          null,
          null,
          state -> SignalLogger.writeString("Elevator Motor SysId", state.toString())),
      new SysIdRoutine.Mechanism((voltage) -> {
        elevatorMotorLeader.setControl(sysIdElevatorControl.withOutput(voltage.in(Volts)));
      }, null, this));

  private final StatusSignal<Boolean> atTopLimitSignal = canDiElevator.getS1Closed();
  private final StatusSignal<Boolean> atBottomLimitSignal = canDiElevator.getS2Closed();

  /**
   * Creates a new ArmSubsystem.
   */
  public ArmSubsystem() {
    var elevatorTalonConfig = new TalonFXConfiguration();
    var armTalonConfig = new TalonFXConfiguration();
    elevatorTalonConfig.MotorOutput.withNeutralMode(Brake);
    elevatorTalonConfig.withSlot0(Slot0Configs.from(SLOT_CONFIGS));
    elevatorTalonConfig.CurrentLimits.withSupplyCurrentLimit(SUPPLY_CURRENT_LIMIT).withSupplyCurrentLimitEnable(true);
    elevatorTalonConfig.withMotionMagic(MOTION_MAGIC_CONFIGS);
    elevatorTalonConfig.HardwareLimitSwitch.withForwardLimitRemoteSensorID(canDiElevator.getDeviceID())
        .withForwardLimitSource(RemoteCANdiS1)
        .withReverseLimitRemoteSensorID(canDiElevator.getDeviceID())
        .withReverseLimitSource(RemoteCANdiS2);
    elevatorTalonConfig.SoftwareLimitSwitch.withForwardSoftLimitEnable(true)
        .withForwardSoftLimitThreshold(TOP_LIMIT.in(Meters) / METERS_PER_REVOLUTION.in(Meters))
        .withReverseSoftLimitEnable(true)
        .withReverseSoftLimitThreshold(BOTTOM_LIMIT.in(Meters) / METERS_PER_REVOLUTION.in(Meters));

    elevatorMotorLeader.getConfigurator().apply(elevatorTalonConfig);
    elevatorMotorFollower.getConfigurator().apply(elevatorTalonConfig);
    elevatorMotorFollower.setControl(new Follower(elevatorMotorLeader.getDeviceID(), false));

    armTalonConfig.MotorOutput.withNeutralMode(Brake);
    armTalonConfig.withSlot0(Slot0Configs.from(SLOT_CONFIGS));
    armTalonConfig.CurrentLimits.withSupplyCurrentLowerLimit(SUPPLY_CURRENT_LIMIT); // a
    armTalonConfig.withMotionMagic(MOTION_MAGIC_CONFIGS);
    armTalonConfig.HardwareLimitSwitc.withForwardLimitRemoteSensorID()


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
        .setControl(elevatorControl.withPosition(position.in(Meters) * METERS_PER_REVOLUTION.in(Meters)));
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

  public void orientToLevel2() {
    orientToAngle(LEVEL_2_ANGLE);
  }

  public void orientToLevel3() {
    orientToAngle(LEVEL_3_ANGLE);
  }

  public void orientToLevel4() {
    orientToAngle(LEVEL_4_ANGLE);
  }

  public void orientForIntake() {
    orientToAngle(INTAKE_ANGLE);
  }

  public void orientForHolding() {
    orientToAngle(HOLDING_ANGLE);
  }

  private void orientToAngle(Angle targetAngle) {

  }
}
