package frc.robot.subsystems;

import static com.ctre.phoenix6.signals.NeutralModeValue.Brake;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Constants.CANIVORE_BUS_NAME;
import static frc.robot.Constants.IndexerConstants.DEVICE_ID_BELT;
import static frc.robot.Constants.IndexerConstants.EJECT_VELOCITY;
import static frc.robot.Constants.IndexerConstants.INDEXER_SPEED_TOLERANCE;
import static frc.robot.Constants.IndexerConstants.INTAKE_VELOCITY;
import static frc.robot.Constants.IndexerConstants.SCORE_VELOCITY_LEVEL_1;
import static frc.robot.Constants.IndexerConstants.SLOT_CONFIGS;
import static frc.robot.Constants.IndexerConstants.SUPPLY_CURRENT_LIMIT;
import static frc.robot.Constants.IndexerConstants.TORQUE_CURRENT_LIMIT;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

/**
 * The subsystem for the coral box/indexer
 */
public class IndexerSubsystem extends SubsystemBase {

  private final TalonFX beltMotor = new TalonFX(DEVICE_ID_BELT, CANIVORE_BUS_NAME);
  private final VelocityTorqueCurrentFOC beltControl = new VelocityTorqueCurrentFOC(0.0);
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

  public IndexerSubsystem() {
    var beltTalonConfig = new TalonFXConfiguration();
    beltTalonConfig.MotorOutput.withNeutralMode(Brake);
    beltTalonConfig.withSlot0(Slot0Configs.from(SLOT_CONFIGS));
    beltTalonConfig.CurrentLimits.withSupplyCurrentLimit(SUPPLY_CURRENT_LIMIT).withSupplyCurrentLimitEnable(true);
    beltTalonConfig.TorqueCurrent.withPeakForwardTorqueCurrent(TORQUE_CURRENT_LIMIT)
        .withPeakReverseTorqueCurrent(TORQUE_CURRENT_LIMIT.unaryMinus());
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
    beltMotor.setControl(beltControl.withVelocity(INTAKE_VELOCITY));
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
    beltMotor.setControl(beltControl.withVelocity(SCORE_VELOCITY_LEVEL_1));
  }

  /**
   * Run belt backward to remove coral from the system
   */
  public void eject() {
    beltMotor.setControl(beltControl.withVelocity(EJECT_VELOCITY));
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
}
