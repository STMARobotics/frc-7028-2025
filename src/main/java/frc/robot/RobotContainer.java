// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction.kForward;
import static edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction.kReverse;
import static frc.robot.Constants.AlignmentConstants.REEF_BRANCH_POSES_BLUE;
import static frc.robot.Constants.AlignmentConstants.REEF_BRANCH_POSES_RED;
import static frc.robot.Constants.TeleopDriveConstants.MAX_TELEOP_ANGULAR_VELOCITY;
import static frc.robot.Constants.TeleopDriveConstants.MAX_TELEOP_VELOCITY;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.EjectCoralCommand;
import frc.robot.commands.IntakeCoralCommand;
import frc.robot.commands.PhotonVisionCommand;
import frc.robot.commands.TuneArmCommand;
import frc.robot.commands.led.DefaultLEDCommand;
import frc.robot.commands.led.LEDBootAnimationCommand;
import frc.robot.controls.ControlBindings;
import frc.robot.controls.JoystickControlBindings;
import frc.robot.controls.XBoxControlBindings;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.AlignmentSubsystem;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.GamePieceManipulatorSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.MitoCANdriaSubsytem;

@Logged(strategy = Logged.Strategy.OPT_IN)
public class RobotContainer {

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MAX_TELEOP_VELOCITY.times(0.05))
      .withRotationalDeadband(MAX_TELEOP_ANGULAR_VELOCITY.times(0.05))
      .withDriveRequestType(DriveRequestType.Velocity)
      .withSteerRequestType(SteerRequestType.MotionMagicExpo);
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();

  @Logged
  private final IndexerSubsystem indexerSubsystem = new IndexerSubsystem();
  @Logged
  private final GamePieceManipulatorSubsystem gamePieceManipulatorSubsystem = new GamePieceManipulatorSubsystem();
  @Logged
  private final ArmSubsystem armSubsystem = new ArmSubsystem();
  private final ClimbSubsystem climbSubsystem = new ClimbSubsystem();
  private final LEDSubsystem ledSubsystem = new LEDSubsystem();
  @Logged
  private final MitoCANdriaSubsytem mitoCANdriaSubsytem = new MitoCANdriaSubsytem();
  @Logged
  private final AlignmentSubsystem alignmentSubsystem = new AlignmentSubsystem();

  private final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
  private final DrivetrainTelemetry drivetrainTelemetry = new DrivetrainTelemetry();
  private final PhotonVisionCommand visionCommand = new PhotonVisionCommand(drivetrain::addVisionMeasurement);
  private final Command slowModeCommand;
  private final AutoCommands autoCommands = new AutoCommands(
      drivetrain,
      armSubsystem,
      alignmentSubsystem,
      gamePieceManipulatorSubsystem,
      indexerSubsystem);

  private final TestMode testMode = new TestMode(
      gamePieceManipulatorSubsystem,
      climbSubsystem,
      armSubsystem,
      indexerSubsystem);

  /* Path follower */
  private final SendableChooser<Command> autoChooser;
  private final ControlBindings controlBindings;

  public RobotContainer() {
    // Configure control binding scheme
    if (DriverStation.getJoystickIsXbox(0) || Robot.isSimulation()) {
      controlBindings = new XBoxControlBindings();
    } else {
      controlBindings = new JoystickControlBindings();
    }

    // Configure PathPlanner
    NamedCommands.registerCommand("scoreCoralLevel4", autoCommands.autoScoreCoralLevel4());
    NamedCommands.registerCommand("scoreCoralLevel3", autoCommands.autoScoreCoralLevel3());
    autoChooser = AutoBuilder.buildAutoChooser("Tests");
    SmartDashboard.putData("Auto Mode", autoChooser);

    // Configure controls
    slowModeCommand = drivetrain
        .applyRequest(
            () -> drive.withVelocityX(controlBindings.translationX().get().div(2))
                .withVelocityY(controlBindings.translationY().get().div(2))
                .withRotationalRate(controlBindings.omega().get().div(2)))
        .finallyDo(() -> drivetrain.setControl(new SwerveRequest.Idle()));

    configureBindings();

    // Set up default and background commands
    visionCommand.schedule();
    armSubsystem.setDefaultCommand(armSubsystem.run(armSubsystem::park).finallyDo(armSubsystem::stop));
    gamePieceManipulatorSubsystem.setDefaultCommand(
        gamePieceManipulatorSubsystem.run(gamePieceManipulatorSubsystem::activeHoldGamePiece)
            .finallyDo(gamePieceManipulatorSubsystem::stop));
    ledSubsystem.setDefaultCommand(new DefaultLEDCommand(ledSubsystem));
    new LEDBootAnimationCommand(ledSubsystem).schedule();

    // Send the reef poses to the dashboard for debugging
    NetworkTableInstance.getDefault()
        .getTable("reef_blue")
        .getStructArrayTopic("branches", Pose2d.struct)
        .publish()
        .set(REEF_BRANCH_POSES_BLUE.toArray(new Pose2d[REEF_BRANCH_POSES_BLUE.size()]));

    NetworkTableInstance.getDefault()
        .getTable("reef_red")
        .getStructArrayTopic("branches", Pose2d.struct)
        .publish()
        .set(REEF_BRANCH_POSES_RED.toArray(new Pose2d[REEF_BRANCH_POSES_RED.size()]));
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

    controlBindings.climb()
        .ifPresent(
            trigger -> trigger
                .whileTrue(climbSubsystem.run(() -> climbSubsystem.climb()).finallyDo(climbSubsystem::stop)));

    drivetrain.registerTelemetry(drivetrainTelemetry::telemeterize);

    // Coral bindings
    controlBindings.intakeCoral()
        .ifPresent(
            trigger -> trigger
                .onTrue(new IntakeCoralCommand(indexerSubsystem, gamePieceManipulatorSubsystem, armSubsystem)));
    controlBindings.ejectCoral()
        .ifPresent(
            trigger -> trigger
                .whileTrue(new EjectCoralCommand(gamePieceManipulatorSubsystem, armSubsystem, indexerSubsystem)));

    controlBindings.moveArmToReefLevel2()
        .ifPresent(
            trigger -> trigger.onTrue(armSubsystem.run(armSubsystem::moveToLevel2).finallyDo(armSubsystem::stop)));
    controlBindings.moveArmToReefLevel3()
        .ifPresent(
            trigger -> trigger.onTrue(armSubsystem.run(armSubsystem::moveToLevel3).finallyDo(armSubsystem::stop)));
    controlBindings.moveArmToReefLevel4()
        .ifPresent(
            trigger -> trigger.onTrue(armSubsystem.run(armSubsystem::moveToLevel4).finallyDo(armSubsystem::stop)));
    controlBindings.releaseCoral()
        .ifPresent(
            trigger -> trigger.whileTrue(
                gamePieceManipulatorSubsystem.run(gamePieceManipulatorSubsystem::scoreCoral)
                    .finallyDo(gamePieceManipulatorSubsystem::stop)));

    controlBindings.parkArm()
        .ifPresent(trigger -> trigger.onTrue(armSubsystem.runOnce(armSubsystem::park).finallyDo(armSubsystem::stop)));

    controlBindings.tuneArm().ifPresent(trigger -> trigger.onTrue(new TuneArmCommand(armSubsystem)));

    controlBindings.moveArmToReefAlgaeLevel1()
        .ifPresent(
            trigger -> trigger.onTrue(
                armSubsystem.run(armSubsystem::moveToAlgaeLevel1)
                    .alongWith(gamePieceManipulatorSubsystem.run(gamePieceManipulatorSubsystem::intakeAlgae))
                    .finallyDo(() -> {
                      armSubsystem.stop();
                      gamePieceManipulatorSubsystem.stop();
                    })));
    controlBindings.moveArmToReefAlgaeLevel2()
        .ifPresent(
            trigger -> trigger.onTrue(
                armSubsystem.run(armSubsystem::moveToAlgaeLevel2)
                    .alongWith(gamePieceManipulatorSubsystem.run(gamePieceManipulatorSubsystem::intakeAlgae))
                    .finallyDo(() -> {
                      armSubsystem.stop();
                      gamePieceManipulatorSubsystem.stop();
                    })));

    controlBindings.ejectAlgae()
        .ifPresent(
            trigger -> trigger.whileTrue(
                gamePieceManipulatorSubsystem.run(gamePieceManipulatorSubsystem::ejectAlgae)
                    .alongWith(armSubsystem.run(() -> armSubsystem.moveToPosition(Meters.zero(), Rotations.of(0.5))))
                    .finallyDo(() -> {
                      gamePieceManipulatorSubsystem.stop();
                      armSubsystem.stop();
                    })));

    controlBindings.slowMode().ifPresent(trigger -> trigger.whileTrue(slowModeCommand));

    controlBindings.scoreCoralLevel3().ifPresent(trigger -> trigger.whileTrue(autoCommands.autoScoreCoralLevel3()));
    controlBindings.scoreCoralLevel4().ifPresent(trigger -> trigger.whileTrue(autoCommands.autoScoreCoralLevel4()));
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
    SmartDashboard.putData("Arm in coast", armSubsystem.runOnce(armSubsystem::coast).ignoringDisable(true));
    SmartDashboard.putData("Arm in brake", armSubsystem.runOnce(armSubsystem::brake).ignoringDisable(true));

    // SysID commands

    // Drive
    SmartDashboard.putData("Drive Quasi Fwd", drivetrain.sysIdTranslationQuasiCommand(kForward));
    SmartDashboard.putData("Drive Quasi Rev", drivetrain.sysIdTranslationQuasiCommand(kReverse));
    SmartDashboard.putData("Drive Dynam Fwd", drivetrain.sysIdTranslationDynamCommand(kForward));
    SmartDashboard.putData("Drive Dynam Rev", drivetrain.sysIdTranslationDynamCommand(kReverse));

    // Drive TorqueFOC
    SmartDashboard.putData("Drive Torque Quasi Fwd", drivetrain.sysIdTranslationQuasiTorqueCommand(kForward));
    SmartDashboard.putData("Drive Torque Quasi Rev", drivetrain.sysIdTranslationQuasiTorqueCommand(kReverse));
    SmartDashboard.putData("Drive Torque Dynam Fwd", drivetrain.sysIdTranslationDynamTorqueCommand(kForward));
    SmartDashboard.putData("Drive Torque Dynam Rev", drivetrain.sysIdTranslationDynamTorqueCommand(kReverse));

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