package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class arm implements Subsystem {
  private final TalonFX motorA = new TalonFX(0);
  private final TalonFX motorB = new TalonFX(0);

  private int targetLevel;
  private boolean atTargetLevel = false;
  private boolean active = false;

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
}
