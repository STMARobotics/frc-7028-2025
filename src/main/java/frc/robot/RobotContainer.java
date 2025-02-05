// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction.kForward;
import static edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction.kReverse;
import static frc.robot.Constants.TeleopDriveConstants.MAX_TELEOP_ANGULAR_VELOCITY;
import static frc.robot.Constants.TeleopDriveConstants.MAX_TELEOP_VELOCITY;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.IntakeCoralCommand;
import frc.robot.commands.PhotonVisionCommand;
import frc.robot.controls.ControlBindings;
import frc.robot.controls.XBoxControlBindings;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.GamePieceManipulatorSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.MitoCANdriaSubsytem;

@Logged(strategy = Logged.Strategy.OPT_IN)
public class RobotContainer {

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MAX_TELEOP_VELOCITY.times(0.1)) // Add a 10% deadband
      .withRotationalDeadband(MAX_TELEOP_ANGULAR_VELOCITY.times(0.1))
      .withDriveRequestType(DriveRequestType.Velocity)
      .withSteerRequestType(SteerRequestType.MotionMagicExpo);
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();

  private final IndexerSubsystem indexerSubsystem = new IndexerSubsystem();
  private final GamePieceManipulatorSubsystem gamePieceManipulatorSubsystem = new GamePieceManipulatorSubsystem();
  private final ArmSubsystem armSubsystem = new ArmSubsystem();
  private final ClimbSubsystem climbSubsystem = new ClimbSubsystem();
  @Logged
  private final MitoCANdriaSubsytem mitoCANdriaSubsytem = new MitoCANdriaSubsytem();

  private final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
  private final DrivetrainTelemetry drivetrainTelemetry = new DrivetrainTelemetry();
  private final PhotonVisionCommand visionCommand = new PhotonVisionCommand(drivetrain::addVisionMeasurement);

  private final TestMode testMode = new TestMode(
      gamePieceManipulatorSubsystem,
      climbSubsystem,
      armSubsystem,
      indexerSubsystem);

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

    controlBindings.wheelsToX().ifPresent(trigger -> trigger.onTrue(drivetrain.applyRequest(() -> brake)));
    controlBindings.seedFieldCentric()
        .ifPresent(trigger -> trigger.onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric)));

    drivetrain.registerTelemetry(drivetrainTelemetry::telemeterize);

    controlBindings.intakeCoral()
        .ifPresent(
            trigger -> trigger.whileTrue(new IntakeCoralCommand(indexerSubsystem, gamePieceManipulatorSubsystem)));
  }

  public Command getAutonomousCommand() {
    /* Run the path selected from the auto chooser */
    return autoChooser.getSelected();
  }

  public void populateSysIdDashboard() {
    var tab = Shuffleboard.getTab("SysId");

    // Drive
    tab.add("Drive Quasi Fwd", drivetrain.sysIdTranslationQuasiCommand(kForward));
    tab.add("Drive Quasi Rev", drivetrain.sysIdTranslationQuasiCommand(kReverse));
    tab.add("Drive Dynam Fwd", drivetrain.sysIdTranslationDynamCommand(kForward));
    tab.add("Drive Dynam Rev", drivetrain.sysIdTranslationDynamCommand(kReverse));
    tab.add("Slip Test", drivetrain.sysIdDriveSlipCommand());

    // Steer
    tab.add("Steer Quasi Fwd", drivetrain.sysIdSteerQuasiCommand(kForward));
    tab.add("Steer Quasi Rev", drivetrain.sysIdSteerQuasiCommand(kReverse));
    tab.add("Steer Dynam Fwd", drivetrain.sysIdSteerDynamCommand(kForward));
    tab.add("Steer Dynam Rev", drivetrain.sysIdSteerDynamCommand(kReverse));

    // Rotation
    tab.add("Rotate Quasi Fwd", drivetrain.sysIdRotationQuasiCommand(kForward));
    tab.add("Rotate Quasi Rev", drivetrain.sysIdRotationQuasiCommand(kReverse));
    tab.add("Rotate Dynam Fwd", drivetrain.sysIdRotationDynamCommand(kForward));
    tab.add("Rotate Dynam Rev", drivetrain.sysIdRotationDynamCommand(kReverse));

    // Indexer
    tab.add("Indexer Quasi Forward", indexerSubsystem.sysIdBeltQuasistaticCommand(kForward));
    tab.add("Indexer Quasi Reverse", indexerSubsystem.sysIdBeltQuasistaticCommand(kReverse));
    tab.add("Indexer Dynam Forward", indexerSubsystem.sysIdBeltDynamicCommand(kForward));
    tab.add("Indexer Dynam Reverse", indexerSubsystem.sysIdBeltDynamicCommand(kReverse));

    // Elevator
    tab.add("Elevator Quasi Forward", armSubsystem.sysIdElevatorQuasistaticCommand(kForward));
    tab.add("Elevator Quasi Reverse", armSubsystem.sysIdElevatorQuasistaticCommand(kReverse));
    tab.add("Elevator Dynam Forward", armSubsystem.sysIdElevatorDynamicCommand(kForward));
    tab.add("Elevator Dynam Reverse", armSubsystem.sysIdElevatorDynamicCommand(kReverse));

    // Arm
    tab.add("Arm Quasi Forward", armSubsystem.sysIdArmQuasistaticCommand(kForward));
    tab.add("Arm Quasi Reverse", armSubsystem.sysIdArmQuasistaticCommand(kReverse));
    tab.add("Arm Dynam Forward", armSubsystem.sysIdArmDynamicCommand(kForward));
    tab.add("Arm Dynam Reverse", armSubsystem.sysIdArmDynamicCommand(kReverse));

    // Manipulator
    tab.add("Manipulator Quasi Forward", gamePieceManipulatorSubsystem.sysIdManipulatorQuasistaticCommand(kForward));
    tab.add("Manipulator Quasi Reverse", gamePieceManipulatorSubsystem.sysIdManipulatorQuasistaticCommand(kReverse));
    tab.add("Manipulator Dynam Forward", gamePieceManipulatorSubsystem.sysIdManipulatorDynamicCommand(kForward));
    tab.add("Manipulator Dynam Reverse", gamePieceManipulatorSubsystem.sysIdManipulatorDynamicCommand(kReverse));
  }

  /**
   * Creates the testing tab in Elastic along with all the test result displays
   */
  public void populateTestingDashboard() {
    var tab = Shuffleboard.getTab("Testing");

    tab.add("Run Tests", testMode.testCommand());
    tab.addBoolean("Indexer Fowards Test", () -> testMode.getIndexerForwardsTestResult());
    tab.addBoolean("Indexer Backwards Test", () -> testMode.getIndexerBackwardsTestResult());
    tab.addBoolean("Manipulator Forwards Test", () -> testMode.getManipulatorForwardsTestResult());
    tab.addBoolean("Manipulator Backwards Test", () -> testMode.getManipulatorBackwardsTestResult());
    tab.addBoolean("Arm Elevator Test", () -> testMode.getArmElevatorTestResult());
    tab.addBoolean("Arm Test", () -> testMode.getArmTestResult());
  }
}
