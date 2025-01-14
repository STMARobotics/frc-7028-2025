package frc.robot.subsystems;

import static frc.robot.Constants.IndexerConstants.DEVICE_ID_BELT;

import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;

/*
 * The subsystem for the coral box/indexer
 */
public class IndexerSubsystem implements Subsystem {

  private final TalonFX beltMotor = new TalonFX(DEVICE_ID_BELT);
  private final VelocityTorqueCurrentFOC beltControl = new VelocityTorqueCurrentFOC(0.0);

  public IndexerSubsystem() {

  }

  /*
   * Runs belt to move coral onto end effector
   */
  public void intake() {
    beltMotor.setControl(beltControl.withVelocity(Constants.IndexerConstants.INTAKE_VELOCITY));
  }

  /*
   * Stops the belt
   */
  public void stop() {
    beltMotor.stopMotor();
  }

  /*
   * Run belt backward to score on L1
   */
  public void scoreL1() {
    beltMotor.setControl(beltControl.withVelocity(Constants.IndexerConstants.SCOREL1_VELOCITY));
  }

  /*
   * Run belt backward to remove coral from the system
   */
  public void eject() {
    beltMotor.setControl(beltControl.withVelocity(Constants.IndexerConstants.EJECT_VELOCITY));
  }
}
