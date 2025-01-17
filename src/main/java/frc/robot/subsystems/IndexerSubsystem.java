package frc.robot.subsystems;

import static frc.robot.Constants.IndexerConstants.DEVICE_ID_BELT;
import static frc.robot.Constants.IndexerConstants.EJECT_VELOCITY;
import static frc.robot.Constants.IndexerConstants.INTAKE_VELOCITY;
import static frc.robot.Constants.IndexerConstants.SCORE_VELOCITY_LEVEL_1;
import static frc.robot.Constants.IndexerConstants.SLOT_CONFIGS;
import static frc.robot.Constants.IndexerConstants.SUPPLY_CURRENT_LIMIT;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

/*
 * The subsystem for the coral box/indexer
 */
public class IndexerSubsystem implements Subsystem {

  private final TalonFX beltMotor = new TalonFX(DEVICE_ID_BELT);
  private final VelocityTorqueCurrentFOC beltControl = new VelocityTorqueCurrentFOC(0.0);
  private final TorqueCurrentFOC beltSysIdControl = new TorqueCurrentFOC(0.0);

  private final SysIdRoutine BeltSysIdRoutine = new SysIdRoutine(
    new SysIdRoutine.Config(null, null, null, state -> SignalLogger.writeString("Indexer Sys ID", state.toString())));

  public IndexerSubsystem() {
    var indexerTalonConfig = new TalonFXConfiguration();
    beltTalonConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    beltTalonConfig.Slot0 = Slot0Configs.from(SLOT_CONFIGS);
    beltTalonConfig.getConfigurator().apply(indexerTalonConfig);
    beltMotor.CurrentLimits.SupplyCurrentLimit = SUPPLY_CURRENT_LIMIT.in(Amps);
    beltMotor.CurrentLimits.SupplyCurrentLimitEnable = true;
  }

  /*
   * Runs belt to move coral onto end effector
   */
  public void intake() {
    beltMotor.setControl(beltControl.withVelocity(INTAKE_VELOCITY));
  }

  /*
   * Stops the belt
   */
  public void stop() {
    beltMotor.stopMotor();
  }

  /*
   * Run belt backward to score on level 1 of the reef
   */
  public void scoreLevel1() {
    beltMotor.setControl(beltControl.withVelocity(SCORE_VELOCITY_LEVEL_1));
  }

  /*
   * Run belt backward to remove coral from the system
   */
  public void eject() {
    beltMotor.setControl(beltControl.withVelocity(EJECT_VELOCITY));
  }
}
