// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction.kForward;
import static edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction.kReverse;
import static frc.robot.Constants.TeleopDriveConstants.MAX_TELEOP_ANGULAR_VELOCITY;
import static frc.robot.Constants.TeleopDriveConstants.MAX_TELEOP_VELOCITY;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.PhotonVisionCommand;
import frc.robot.controls.ControlBindings;
import frc.robot.controls.XBoxControlBindings;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.IndexerSubsystem;

@Logged(strategy = Logged.Strategy.OPT_IN)
public class RobotContainer {

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MAX_TELEOP_VELOCITY.times(0.1)) // Add a 10% deadband
      .withRotationalDeadband(MAX_TELEOP_ANGULAR_VELOCITY.times(0.1))
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();

  private final ArmSubsystem armSubsystem = new ArmSubsystem();
  private final IndexerSubsystem indexerSubsystem = new IndexerSubsystem();
  private final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
  private final DrivetrainTelemetry drivetrainTelemetry = new DrivetrainTelemetry();
  private final PhotonVisionCommand visionCommand = new PhotonVisionCommand(drivetrain::addVisionMeasurement);
  private final TestMode testMode = new TestMode();

  /* Path follower */
  private final SendableChooser<Command> autoChooser;
  private final ControlBindings controlBindings;

  public RobotContainer() {
    autoChooser = AutoBuilder.buildAutoChooser("Tests");
    SmartDashboard.putData("Auto Mode", autoChooser);

    controlBindings = new XBoxControlBindings();
    configureBindings();

    visionCommand.schedule();
  }

  private void configureBindings() {
    // Default drivetrain command for teleop control
    drivetrain.setDefaultCommand(
        drivetrain.applyRequest(
            () -> drive.withVelocityX(controlBindings.translationX().get())
                .withVelocityY(controlBindings.translationY().get())
                .withRotationalRate(controlBindings.omega().get())));

    controlBindings.wheelsToX().ifPresent(trigger -> trigger.whileTrue(drivetrain.applyRequest(() -> brake)));
    controlBindings.seedFieldCentric()
        .ifPresent(trigger -> trigger.onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric)));

    drivetrain.registerTelemetry(drivetrainTelemetry::telemeterize);
  }

  public Command getAutonomousCommand() {
    /* Run the path selected from the auto chooser */
    return autoChooser.getSelected();
  }

  public void indexerPopulateDashboard() {
    var tab = Shuffleboard.getTab("Indexer SysId");
    int columnIndex = 0;

    // Column 0 indexer
    tab.add("Indexer Quasi Forward", indexerSubsystem.sysIdBeltQuasistaticCommand(kForward))
        .withPosition(columnIndex, 0);
    tab.add("Indexer Quasi Reverse", indexerSubsystem.sysIdBeltQuasistaticCommand(kReverse))
        .withPosition(columnIndex, 1);
    tab.add("Indexer Dynam Forward", indexerSubsystem.sysIdBeltDynamicCommand(kForward)).withPosition(columnIndex, 2);
    tab.add("Indexer Dynam Reverse", indexerSubsystem.sysIdBeltDynamicCommand(kReverse)).withPosition(columnIndex, 3);
  }

  public void elevatorPopulateDashboard() {
    var tab = Shuffleboard.getTab("Elevator SysId");
    int columnIndex = 0;

    // Column 1 Elevator motor 1
    tab.add("Elevator Quasi Forward", armSubsystem.sysIdElevatorQuasistaticCommand(kForward))
        .withPosition(columnIndex + 1, 0);
    tab.add("Elevator Quasi Reverse", armSubsystem.sysIdElevatorQuasistaticCommand(kReverse))
        .withPosition(columnIndex + 1, 1);
    tab.add("Elevator Dynam Forward", armSubsystem.sysIdElevatorDynamicCommand(kForward))
        .withPosition(columnIndex + 1, 2);
    tab.add("Elevator Dynam Reverse", armSubsystem.sysIdElevatorDynamicCommand(kReverse))
        .withPosition(columnIndex + 1, 3);
  }

  /**
   * Creates the testing tab in Elastic along with all the test result displays
   */
  public void populateTestingDashboard() {
    SmartDashboard.putData("Run Tests", testMode.testCommand());

    SmartDashboard.putBoolean("Indexer Fowards Test", testMode.getIndexerForwardsTestResult());
    SmartDashboard.putBoolean("Indexer Backwards Test", testMode.getIndexerBackwardsTestResult());
    SmartDashboard.putBoolean("Manipulator Forwards Test", testMode.getManipulatorForwardsTestResult());
    SmartDashboard.putBoolean("Manipulator Backwards Test", testMode.getManipulatorBackwardsTestResult());
    SmartDashboard.putBoolean("Algae Rollers Forwards Test", testMode.getAlgaeRollersForwardsTestResult());
    SmartDashboard.putBoolean("Algae Rollers Backwards Test", testMode.getAlgaeRollersBackwardsTestResult());
    SmartDashboard.putBoolean("Algae Intake Up Test", testMode.getAlgaeIntakeUpTestResult());
    SmartDashboard.putBoolean("Algae Intake Down Test", testMode.getAlgaeIntakeDownTestResult());
    SmartDashboard.putBoolean("Arm Elevator Test", testMode.getArmElevatorTestResult());
    SmartDashboard.putBoolean("Arm Test", testMode.getArmTestResult());
  }
}
