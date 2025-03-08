package frc.robot.subsystems;

import static com.ctre.phoenix6.signals.InvertedValue.Clockwise_Positive;
import static com.ctre.phoenix6.signals.NeutralModeValue.Brake;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Constants.CANIVORE_BUS_NAME;
import static frc.robot.Constants.IndexerConstants.DEVICE_ID_BELT;
import static frc.robot.Constants.IndexerConstants.INDEXER_STATOR_CURRENT_LIMIT;
import static frc.robot.Constants.IndexerConstants.INDEXER_SUPPLY_CURRENT_LIMIT;
import static frc.robot.Constants.IndexerConstants.INDEXER_TORQUE_CURRENT_LIMIT;
import static frc.robot.Constants.IndexerConstants.SLOT_CONFIGS;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
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

  private final VoltageOut beltVoltageOut = new VoltageOut(0.0).withEnableFOC(true);
  private final TorqueCurrentFOC beltSysIdControl = new TorqueCurrentFOC(0.0);

  private final StatusSignal<AngularVelocity> beltVelocitySignal = beltMotor.getVelocity();

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
   * Runs belt to move coral onto end effector
   */
  public void intake() {
    beltMotor.setControl(beltVoltageOut.withOutput(2.0));
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
    beltMotor.setControl(beltVoltageOut.withOutput(-4.0));
  }

  /**
   * Run belt backward to remove coral from the system
   */
  public void eject() {
    beltMotor.setControl(beltVoltageOut.withOutput(-4.0));
  }

  /**
   * Gets the velocity of the belt motor
   * 
   * @return velocity of the belt motor
   */
  public AngularVelocity getBeltVelocity() {
    return beltVelocitySignal.refresh().getValue();
  }
}