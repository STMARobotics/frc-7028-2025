package frc.robot.subsystems;

import static com.ctre.phoenix6.signals.InvertedValue.Clockwise_Positive;
import static com.ctre.phoenix6.signals.NeutralModeValue.Brake;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Constants.CANIVORE_BUS_NAME;
import static frc.robot.Constants.IndexerConstants.CORAL_DETECTION_THRESHOLD;
import static frc.robot.Constants.IndexerConstants.DEVICE_ID_BELT;
import static frc.robot.Constants.IndexerConstants.DEVICE_ID_GAME_PIECE_CANRANGE;
import static frc.robot.Constants.IndexerConstants.INDEXER_SPEED_TOLERANCE;
import static frc.robot.Constants.IndexerConstants.INDEXER_STATOR_CURRENT_LIMIT;
import static frc.robot.Constants.IndexerConstants.INDEXER_SUPPLY_CURRENT_LIMIT;
import static frc.robot.Constants.IndexerConstants.INDEXER_TORQUE_CURRENT_LIMIT;
import static frc.robot.Constants.IndexerConstants.SLOT_CONFIGS;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

/**
 * The subsystem for the coral box/indexer
 */
@Logged
public class IndexerSubsystem extends SubsystemBase {

  private final TalonFX beltMotor = new TalonFX(DEVICE_ID_BELT, CANIVORE_BUS_NAME);
  private final CANrange intakeCanRange = new CANrange(DEVICE_ID_GAME_PIECE_CANRANGE, CANIVORE_BUS_NAME);

  // TODO voltage for week zero
  private final VoltageOut beltVoltageOut = new VoltageOut(0.0).withEnableFOC(true);
  private final VelocityTorqueCurrentFOC beltControl = new VelocityTorqueCurrentFOC(0.0);
  private final TorqueCurrentFOC beltSysIdControl = new TorqueCurrentFOC(0.0);

  private final StatusSignal<AngularVelocity> beltVelocitySignal = beltMotor.getVelocity();
  private final StatusSignal<Boolean> intakeCoralDetected = intakeCanRange.getIsDetected(false);

  private final SysIdRoutine beltSysIdRoutine = new SysIdRoutine(
      new SysIdRoutine.Config(
          Volts.of(5).per(Second),
          null,
          null,
          state -> SignalLogger.writeString("Indexer SysId", state.toString())),
      new SysIdRoutine.Mechanism((amps) -> {
        beltMotor.setControl(beltSysIdControl.withOutput(amps.in(Volts)));
      }, null, this));

  /**
   * Creates a new IndexerSubsystem.
   */
  public IndexerSubsystem() {
    var beltTalonConfig = new TalonFXConfiguration();
    beltTalonConfig.MotorOutput.withNeutralMode(Brake).withInverted(Clockwise_Positive);
    beltTalonConfig.withSlot0(Slot0Configs.from(SLOT_CONFIGS));
    beltTalonConfig.CurrentLimits.withSupplyCurrentLimit(INDEXER_SUPPLY_CURRENT_LIMIT)
        .withSupplyCurrentLimitEnable(true)
        .withStatorCurrentLimit(INDEXER_STATOR_CURRENT_LIMIT)
        .withStatorCurrentLimitEnable(true);
    beltTalonConfig.TorqueCurrent.withPeakForwardTorqueCurrent(INDEXER_TORQUE_CURRENT_LIMIT)
        .withPeakReverseTorqueCurrent(INDEXER_TORQUE_CURRENT_LIMIT.unaryMinus());
    beltMotor.getConfigurator().apply(beltTalonConfig);

    var canRangeConfig = new CANrangeConfiguration();
    canRangeConfig.ProximityParams.withProximityThreshold(CORAL_DETECTION_THRESHOLD);
    intakeCanRange.getConfigurator().apply(canRangeConfig);
  }

  /**
   * Command to run indexer belt SysId routine in dynamic mode
   * 
   * @param direction The direction to run the belt motor for dynamic mode
   * @return The SysId output data for dynamic mode
   */
  public Command sysIdBeltDynamicCommand(Direction direction) {
    return beltSysIdRoutine.dynamic(direction)
        .withName("SysId indexer belt dynamic " + direction)
        .finallyDo(this::stop);
  }

  /**
   * Command to run indexer belt SysId routine in quasistatic mode
   * 
   * @param direction The direction to run the belt motor for quasistatic mode
   * @return The SysId output data for quasistatic mode
   */
  public Command sysIdBeltQuasistaticCommand(Direction direction) {
    return beltSysIdRoutine.quasistatic(direction)
        .withName("SysId indexer belt quasi " + direction)
        .finallyDo(this::stop);
  }

  /**
   * Runs the indexer belt at any specific speed
   * 
   * @param speed speed to run the belt
   */
  public void runBelt(AngularVelocity speed) {
    beltMotor.setControl(beltControl.withVelocity(speed));
  }

  /**
   * Runs belt to move coral onto end effector
   */
  public void intake() {
    // TODO voltage for week zero
    beltMotor.setControl(beltVoltageOut.withOutput(2.0));
    // beltMotor.setControl(beltControl.withVelocity(INTAKE_VELOCITY));
  }

  /**
   * Stops the belt
   */
  public void stop() {
    beltMotor.stopMotor();
  }

  /**
   * Run belt backward to score on level 1 of the reef
   */
  public void scoreLevel1() {
    // TODO voltage for week zero
    beltMotor.setControl(beltVoltageOut.withOutput(-4.0));
    // beltMotor.setControl(beltControl.withVelocity(SCORE_VELOCITY_LEVEL_1));
  }

  /**
   * Run belt backward to remove coral from the system
   */
  public void eject() {
    // TODO voltage for week zero
    beltMotor.setControl(beltVoltageOut.withOutput(-8.0));
    // beltMotor.setControl(beltControl.withVelocity(EJECT_VELOCITY));
  }

  /**
   * Checks if the indexer is spinning at the proper speed within a tolerance
   * 
   * @return true if the indexer is spinning at the proper speed
   */
  public boolean isIndexerAtSpeed() {
    return beltVelocitySignal.refresh()
        .getValue()
        .minus(beltControl.getVelocityMeasure())
        .abs(RotationsPerSecond) <= INDEXER_SPEED_TOLERANCE.in(RotationsPerSecond);
  }

  /**
   * Checks if there is an object in front of the game piece sensor
   * 
   * @return true if there is a game piece detected, otherwise false
   */
  public boolean isCoralInPickupPosition() {
    return intakeCoralDetected.refresh().getValue();
  }
}