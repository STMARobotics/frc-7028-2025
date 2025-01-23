package frc.robot.subsystems;

import static com.ctre.phoenix6.signals.NeutralModeValue.Brake;
import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Constants.ArmConstants.BOTTOM_LIMIT;
import static frc.robot.Constants.ArmConstants.DEVICE_ID_CANDI;
import static frc.robot.Constants.ArmConstants.DEVICE_ID_ELEVATOR_MOTOR_1;
import static frc.robot.Constants.ArmConstants.DEVICE_ID_ELEVATOR_MOTOR_2;
import static frc.robot.Constants.ArmConstants.LEVEL_1_HEIGHT;
import static frc.robot.Constants.ArmConstants.LEVEL_2_HEIGHT;
import static frc.robot.Constants.ArmConstants.LEVEL_3_HEIGHT;
import static frc.robot.Constants.ArmConstants.LEVEL_4_HEIGHT;
import static frc.robot.Constants.ArmConstants.METERS_PER_REVOLUTION;
import static frc.robot.Constants.ArmConstants.MOTION_MAGIC_CONFIGS;
import static frc.robot.Constants.ArmConstants.SLOT_CONFIGS;
import static frc.robot.Constants.ArmConstants.SUPPLY_CURRENT_LIMIT;
import static frc.robot.Constants.ArmConstants.TOP_LIMIT;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANdi;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

public class ArmSubsystem implements Subsystem {
  private final TalonFX motorA = new TalonFX(0);
  private final TalonFX motorB = new TalonFX(0);

  private final TalonFX elevatorMotor1 = new TalonFX(DEVICE_ID_ELEVATOR_MOTOR_1);
  private final TalonFX elevatorMotor2 = new TalonFX(DEVICE_ID_ELEVATOR_MOTOR_2);

  private final CANdi CANdi = new CANdi(DEVICE_ID_CANDI);

  private final MotionMagicVoltage ElevatorControl = new MotionMagicVoltage(0.0);
  private final VoltageOut voltageControl = new VoltageOut(0.0);

  private final SysIdRoutine elevatorSysIdRoutine1 = new SysIdRoutine(
      new SysIdRoutine.Config(
          null,
          null,
          null,
          state -> SignalLogger.writeString("Elevator Motor 1 SysId", state.toString())),
      new SysIdRoutine.Mechanism((amps) -> {
        elevatorMotor1.setControl(voltageControl.withOutput(amps.in(Volts)));
      }, null, this));

  private final SysIdRoutine elevatorSysIdRoutine2 = new SysIdRoutine(
      new SysIdRoutine.Config(
          null,
          null,
          null,
          state -> SignalLogger.writeString("Elevator Motor 2 SysId", state.toString())),
      new SysIdRoutine.Mechanism((amps) -> {
        elevatorMotor2.setControl(voltageControl.withOutput(amps.in(Volts)));
      }, null, this));

  private int targetLevel;
  private boolean atTargetLevel = false;
  private boolean active = false;

  public ArmSubsystem() {
    var ElevatorTalonConfig = new TalonFXConfiguration();
    ElevatorTalonConfig.MotorOutput.NeutralMode = Brake;
    ElevatorTalonConfig.Slot0 = Slot0Configs.from(SLOT_CONFIGS);
    ElevatorTalonConfig.CurrentLimits.SupplyCurrentLimit = SUPPLY_CURRENT_LIMIT.in(Amps);
    ElevatorTalonConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    ElevatorTalonConfig.MotionMagic = MOTION_MAGIC_CONFIGS;
    ElevatorTalonConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    ElevatorTalonConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = TOP_LIMIT.in(Meters)
        / METERS_PER_REVOLUTION.in(Meters);
    ElevatorTalonConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    ElevatorTalonConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = BOTTOM_LIMIT.in(Meters)
        / METERS_PER_REVOLUTION.in(Meters);

    elevatorMotor1.getConfigurator().apply(ElevatorTalonConfig);
    elevatorMotor2.getConfigurator().apply(ElevatorTalonConfig);
  }

  /**
   * Command to run Elevator 1 motor SysId routine in dynamic mode
   * 
   * @param direction The direction to run the elevator motor for dynamic mode
   * @return The SysId output data for dynamic mode
   */
  public Command sysIdElevator1DynamicCommand(Direction direction) {
    return elevatorSysIdRoutine1.dynamic(direction)
        .withName("SysId elevator 1 dynamic " + direction)
        .finallyDo(this::stopElevator);
  }

  /**
   * Command to run Elevator 1 motor SysId routine in quasistatic mode
   * 
   * @param direction The direction to run the elevator motor for quasistatic mode
   * @return The SysId output data for quasistatic mode
   */
  public Command sysIdElevator1QuasistaticCommand(Direction direction) {
    return elevatorSysIdRoutine1.quasistatic(direction)
        .withName("SysId elevator 1 quasi " + direction)
        .finallyDo(this::stopElevator);
  }

  /**
   * Command to run Elevator 2 motor SysId routine in dynamic mode
   * 
   * @param direction The direction to run the elevator motor for dynamic mode
   * @return The SysId output data for dynamic mode
   */
  public Command sysIdElevator2DynamicCommand(Direction direction) {
    return elevatorSysIdRoutine2.dynamic(direction)
        .withName("SysId elevator 2 dynamic " + direction)
        .finallyDo(this::stopElevator);
  }

  /**
   * Command to run Elevator 2 motor SysId routine in quasistatic mode
   * 
   * @param direction The direction to run the elevator motor for quasistatic mode
   * @return The SysId output data for quasistatic mode
   */
  public Command sysIdElevator2QuasistaticCommand(Direction direction) {
    return elevatorSysIdRoutine2.quasistatic(direction)
        .withName("SysId elevator 2 quasi " + direction)
        .finallyDo(this::stopElevator);
  }

  /**
   * Indicates if the top limit switch is tripped
   * 
   * @return true if the limit switch is tripped, otherwise false
   */
  public boolean isAtTopLimit() {
    return CANdi.getS1Closed().getValue().booleanValue();
  }

  /**
   * Indicates if the bottom limit switch is tripped
   * 
   * @return true if the bottom limit switch is tripped, otherwise false
   */
  public boolean isAtBottomLimit() {
    return CANdi.getS2Closed().getValue().booleanValue();
  }

  /*
   * Moves the elevator to a position measured in meters.
   * @param The desired position in meters
   */
  public void MoveElevator(Distance position) {
    elevatorMotor1.setControl(
        ElevatorControl.withPosition(position.in(Meters) * METERS_PER_REVOLUTION.in(Meters))
            .withLimitForwardMotion(isAtBottomLimit())
            .withLimitReverseMotion(isAtBottomLimit()));
  }

  /*
   * Moves the elevator to the height required to score on the bottom level
   */
  public void ElevatorLevel1() {
    MoveElevator(LEVEL_1_HEIGHT);
  }

  /*
   * Moves the elevator to the height required to score on the second level
   */
  public void ElevatorLevel2() {
    MoveElevator(LEVEL_2_HEIGHT);
  }

  /*
   * Moves the elevator to the height required to score on the third level
   */
  public void ElevatorLevel3() {
    MoveElevator(LEVEL_3_HEIGHT);
  }

  /*
   * Moves the elevator to the height required to score on the top level
   */
  public void ElevatorLevel4() {
    MoveElevator(LEVEL_4_HEIGHT);
  }

  public void toLevel2() {
    return;
  }

  public void toLevel3() {

  }

  public void toLevel4() {

  }

  public void toIntakePosition() {

  }

  public Boolean isReady() {
    return this.atTargetLevel;
  }

  private void orientToLevel(Integer targetLevel) {
    if (this.active) {
      // the robot should not try to move while active... what should be done in the case this occurs?
    } else {
      this.targetLevel = targetLevel;
    }
  }

  /**
   * Stops the elevator
   */
  public void stopElevator() {
    elevatorMotor1.stopMotor();
    elevatorMotor2.stopMotor();
  }
}
