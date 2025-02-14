package frc.robot.commands;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;

/**
 * A command to help tuning scoring positions, not intended for competition use.
 */
public class TuneArmCommand extends Command {

  private final ArmSubsystem armSubsystem;
  private final DoubleEntry angleEntry;
  private final DoubleEntry heightEntry;

  private final MutAngle angleMeasure = Rotations.mutable(0.0);
  private final MutDistance heightMeasure = Meters.mutable(0.0);

  public TuneArmCommand(ArmSubsystem armSubsystem) {
    this.armSubsystem = armSubsystem;

    // Get the NT entries
    var tuneTable = NetworkTableInstance.getDefault().getTable("Tune Arm");
    angleEntry = tuneTable.getDoubleTopic("angle").getEntry(0);
    heightEntry = tuneTable.getDoubleTopic("height").getEntry(0);

    // Make sure the topics are published
    angleEntry.set(angleEntry.get(0.0));
    heightEntry.set(angleEntry.get(0.0));

    addRequirements(armSubsystem);
  }

  @Override
  public void execute() {
    armSubsystem.moveToPosition(
        heightMeasure.mut_replace(heightEntry.get(0.0), Meters),
          angleMeasure.mut_replace(angleEntry.get(0.0), Rotations));
  }

  @Override
  public void end(boolean interrupted) {
    armSubsystem.stop();
  }

}
