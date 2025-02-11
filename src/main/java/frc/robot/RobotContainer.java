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

    controlBindings.wheelsToX().ifPresent(trigger -> trigger.whileTrue(drivetrain.applyRequest(() -> brake)));
    controlBindings.seedFieldCentric()
        .ifPresent(trigger -> trigger.onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric)));

    if (controlBindings.frontClimb().isPresent() || controlBindings.backClimb().isPresent()) {
      climbSubsystem.setDefaultCommand(climbSubsystem.run(() -> {
        controlBindings.frontClimb()
            .ifPresent(frontSupplier -> climbSubsystem.runFrontClimb(frontSupplier.getAsDouble()));
        controlBindings.backClimb().ifPresent(backSupplier -> climbSubsystem.runBackClimb(backSupplier.getAsDouble()));
      }));
    }

    drivetrain.registerTelemetry(drivetrainTelemetry::telemeterize);

    controlBindings.intakeCoral()
        .ifPresent(
            trigger -> trigger.toggleOnTrue(new IntakeCoralCommand(indexerSubsystem, gamePieceManipulatorSubsystem)));
  }

  public Command getAutonomousCommand() {
    /* Run the path selected from the auto chooser */
    return autoChooser.getSelected();
  }

  /**
   * Sends all of the testing commands to the dashboard
   */
  public void populateTestModeDashboard() {
    // Test mode command
    SmartDashboard.putData("Run Tests", testMode.testCommand());

    // SysID commands

    // Drive
    SmartDashboard.putData("Drive Quasi Fwd", drivetrain.sysIdTranslationQuasiCommand(kForward));
    SmartDashboard.putData("Drive Quasi Rev", drivetrain.sysIdTranslationQuasiCommand(kReverse));
    SmartDashboard.putData("Drive Dynam Fwd", drivetrain.sysIdTranslationDynamCommand(kForward));
    SmartDashboard.putData("Drive Dynam Rev", drivetrain.sysIdTranslationDynamCommand(kReverse));
    SmartDashboard.putData("Slip Test", drivetrain.sysIdDriveSlipCommand());

    // Steer
    SmartDashboard.putData("Steer Quasi Fwd", drivetrain.sysIdSteerQuasiCommand(kForward));
    SmartDashboard.putData("Steer Quasi Rev", drivetrain.sysIdSteerQuasiCommand(kReverse));
    SmartDashboard.putData("Steer Dynam Fwd", drivetrain.sysIdSteerDynamCommand(kForward));
    SmartDashboard.putData("Steer Dynam Rev", drivetrain.sysIdSteerDynamCommand(kReverse));

    // Rotation
    SmartDashboard.putData("Rotate Quasi Fwd", drivetrain.sysIdRotationQuasiCommand(kForward));
    SmartDashboard.putData("Rotate Quasi Rev", drivetrain.sysIdRotationQuasiCommand(kReverse));
    SmartDashboard.putData("Rotate Dynam Fwd", drivetrain.sysIdRotationDynamCommand(kForward));
    SmartDashboard.putData("Rotate Dynam Rev", drivetrain.sysIdRotationDynamCommand(kReverse));

    // Indexer
    SmartDashboard.putData("Indexer Quasi Forward", indexerSubsystem.sysIdBeltQuasistaticCommand(kForward));
    SmartDashboard.putData("Indexer Quasi Reverse", indexerSubsystem.sysIdBeltQuasistaticCommand(kReverse));
    SmartDashboard.putData("Indexer Dynam Forward", indexerSubsystem.sysIdBeltDynamicCommand(kForward));
    SmartDashboard.putData("Indexer Dynam Reverse", indexerSubsystem.sysIdBeltDynamicCommand(kReverse));

    // Elevator
    SmartDashboard.putData("Elevator Quasi Forward", armSubsystem.sysIdElevatorQuasistaticCommand(kForward));
    SmartDashboard.putData("Elevator Quasi Reverse", armSubsystem.sysIdElevatorQuasistaticCommand(kReverse));
    SmartDashboard.putData("Elevator Dynam Forward", armSubsystem.sysIdElevatorDynamicCommand(kForward));
    SmartDashboard.putData("Elevator Dynam Reverse", armSubsystem.sysIdElevatorDynamicCommand(kReverse));

    // Arm
    SmartDashboard.putData("Arm Quasi Forward", armSubsystem.sysIdArmQuasistaticCommand(kForward));
    SmartDashboard.putData("Arm Quasi Reverse", armSubsystem.sysIdArmQuasistaticCommand(kReverse));
    SmartDashboard.putData("Arm Dynam Forward", armSubsystem.sysIdArmDynamicCommand(kForward));
    SmartDashboard.putData("Arm Dynam Reverse", armSubsystem.sysIdArmDynamicCommand(kReverse));

    // Manipulator
    SmartDashboard.putData(
        "Manipulator Quasi Forward",
          gamePieceManipulatorSubsystem.sysIdManipulatorQuasistaticCommand(kForward));
    SmartDashboard.putData(
        "Manipulator Quasi Reverse",
          gamePieceManipulatorSubsystem.sysIdManipulatorQuasistaticCommand(kReverse));
    SmartDashboard
        .putData("Manipulator Dynam Forward", gamePieceManipulatorSubsystem.sysIdManipulatorDynamicCommand(kForward));
    SmartDashboard
        .putData("Manipulator Dynam Reverse", gamePieceManipulatorSubsystem.sysIdManipulatorDynamicCommand(kReverse));
  }
}
