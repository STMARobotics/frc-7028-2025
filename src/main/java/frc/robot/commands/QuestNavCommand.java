package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.vision.QuestNav;

/**
 * Command to interface with QuestNav and send telemetry data over NT
 */
public class QuestNavCommand extends Command {

  private final QuestNav questNav;

  private final NetworkTable telemetryTable = NetworkTableInstance.getDefault().getTable("QuestNavTelemetry");
  private final StructPublisher<Pose2d> questPosePub = telemetryTable.getStructTopic("QuestPose", Pose2d.struct)
      .publish();
  private final StructPublisher<Pose2d> robotPosePub = telemetryTable.getStructTopic("RobotPose", Pose2d.struct)
      .publish();

  public QuestNavCommand(QuestNav questNav) {
    this.questNav = questNav;
  }

  @Override
  public void execute() {
    questNav.cleanUpQuestNavMessages();
    questPosePub.set(questNav.getQuestPose());
    robotPosePub.set(questNav.getRobotPose());
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }

}
