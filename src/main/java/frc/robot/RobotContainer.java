// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.wpilibj.LEDPattern.GradientType.kContinuous;
import static edu.wpi.first.wpilibj.LEDPattern.gradient;
import static edu.wpi.first.wpilibj.LEDPattern.rainbow;
import static edu.wpi.first.wpilibj.LEDPattern.solid;
import static edu.wpi.first.wpilibj.util.Color.kAqua;
import static edu.wpi.first.wpilibj.util.Color.kBlack;
import static edu.wpi.first.wpilibj.util.Color.kBlue;
import static edu.wpi.first.wpilibj.util.Color.kGreenYellow;
import static edu.wpi.first.wpilibj.util.Color.kOrange;
import static edu.wpi.first.wpilibj2.command.Commands.runOnce;
import static edu.wpi.first.wpilibj2.command.Commands.select;
import static edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction.kForward;
import static edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction.kReverse;
import static frc.robot.Constants.TeleopDriveConstants.MAX_TELEOP_ANGULAR_VELOCITY;
import static frc.robot.Constants.TeleopDriveConstants.MAX_TELEOP_VELOCITY;
import static frc.robot.Constants.VisionConstants.CAMERA_NAMES;
import static frc.robot.Constants.VisionConstants.ROBOT_TO_CAMERA_TRANSFORMS;
import static java.util.Map.entry;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.EjectCoralCommand;
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
import java.util.Map;
import org.photonvision.PhotonCamera;

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

  /** The high camera toward the back of the robot, used for scoring on right branches */
  private final PhotonCamera highBackCamera = new PhotonCamera("High-Back");
  /** The high camera toward the front of the robot, used for scoring on left branches */
  private final PhotonCamera highFrontCamera = new PhotonCamera("High-Front");

  private final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
  private final DrivetrainTelemetry drivetrainTelemetry = new DrivetrainTelemetry();
  private final AutoCommands autoCommands = new AutoCommands(
      drivetrain,
      armSubsystem,
      indexerSubsystem,
      alignmentSubsystem,
      gamePieceManipulatorSubsystem,
      indexerSubsystem,
      ledSubsystem,
      highFrontCamera,
      highBackCamera);
  private final ScoreChooser scoreChooser = new ScoreChooser();

  private final TestMode testMode = new TestMode(
      gamePieceManipulatorSubsystem,
      climbSubsystem,
      armSubsystem,
      indexerSubsystem);

  /* Path follower */
  private final SendableChooser<Command> autoChooser;
  private final ControlBindings controlBindings;

  private final Thread photonThread = new Thread(
      new PhotonRunnable(
          CAMERA_NAMES,
          ROBOT_TO_CAMERA_TRANSFORMS,
          drivetrain::addVisionMeasurement,
          () -> drivetrain.getState().Pose));

  public RobotContainer() {
    // Configure control binding scheme
    if (DriverStation.getJoystickIsXbox(0) || Robot.isSimulation()) {
      controlBindings = new XBoxControlBindings();
    } else {
      controlBindings = new JoystickControlBindings();
    }

    // Configure PathPlanner
    NamedCommands.registerCommand("scoreCoralLevel4Right", autoCommands.scoreCoralLevel4Right(true));
    NamedCommands.registerCommand("scoreCoralLevel4Left", autoCommands.scoreCoralLevel4Left(true));
    NamedCommands.registerCommand("intakeCoral", autoCommands.intakeCoral());
    NamedCommands.registerCommand("holdCoral", autoCommands.holdCoral());
    NamedCommands.registerCommand(
        "intakeAndHoldCoral",
          autoCommands.intakeCoral().andThen(autoCommands.holdCoral()).repeatedly());

    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Mode", autoChooser);

    // Configure controls
    configureBindings();

    // Start PhotonVision thread
    photonThread.setName("PhotonVision");
    photonThread.setDaemon(true);
    photonThread.start();

    // Set up default commmands
    var defaultLedCommand = new DefaultLEDCommand(ledSubsystem);
    ledSubsystem.setDefaultCommand(defaultLedCommand);

    // Run IntakeAndHoldCommand when enabled in teleop and no other command is running on arm and intake
    new Trigger(
        () -> RobotState.isEnabled() && RobotState.isTeleop() && armSubsystem.getCurrentCommand() == null
            && indexerSubsystem.getCurrentCommand() == null
            && (ledSubsystem.getCurrentCommand() == null || ledSubsystem.getCurrentCommand() == defaultLedCommand))
        .onTrue(
            autoCommands.intakeCoral()
                .andThen(autoCommands.holdCoral(() -> scoreChooser.getSelectedLevel() == 1))
                .repeatedly());

    // Default to holding coral when nothing else is running. The trigger above WILL interupt this if the arm and intake
    // are not running any command
    gamePieceManipulatorSubsystem
        .setDefaultCommand(gamePieceManipulatorSubsystem.run(gamePieceManipulatorSubsystem::activeHoldCoral));

    // Run the boot animation
    new LEDBootAnimationCommand(ledSubsystem).schedule();
  }

  private Command makeSlowModeCommand() {
    return drivetrain
        .applyRequest(
            () -> drive.withVelocityX(controlBindings.translationX().get().div(2))
                .withVelocityY(controlBindings.translationY().get().div(2))
                .withRotationalRate(controlBindings.omega().get().div(2)))
        .finallyDo(() -> drivetrain.setControl(new SwerveRequest.Idle()));
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
            trigger -> trigger.whileTrue(
                climbSubsystem.run(() -> climbSubsystem.climb())
                    .finallyDo(climbSubsystem::stop)
                    .alongWith(
                        new ScheduleCommand(
                            ledSubsystem.runPatternAsCommand(solid(kGreenYellow))
                                .until(climbSubsystem::isAtLimit)
                                .andThen(
                                    ledSubsystem.runPatternAsCommand(
                                        rainbow(255, 255).scrollAtRelativeSpeed(Percent.per(Second).of(200))))))));

    drivetrain.registerTelemetry(drivetrainTelemetry::telemeterize);

    // Coral bindings
    controlBindings.ejectCoral()
        .ifPresent(
            trigger -> trigger.whileTrue(
                new EjectCoralCommand(gamePieceManipulatorSubsystem, armSubsystem, indexerSubsystem)
                    .alongWith(ledSubsystem.runPatternAsCommand(rainbow(255, 255)))));

    controlBindings.moveArmToReefLevel2()
        .ifPresent(
            trigger -> trigger.toggleOnTrue(
                armSubsystem.run(armSubsystem::moveToLevel2)
                    .alongWith(makeSlowModeCommand())
                    .alongWith(ledSubsystem.runPatternAsCommand(solid(Color.kBlueViolet)))
                    .finallyDo(armSubsystem::stop)));
    controlBindings.moveArmToReefLevel3()
        .ifPresent(
            trigger -> trigger.toggleOnTrue(
                armSubsystem.run(armSubsystem::moveToLevel3)
                    .alongWith(makeSlowModeCommand())
                    .alongWith(ledSubsystem.runPatternAsCommand(solid(kOrange)))
                    .finallyDo(armSubsystem::stop)));
    controlBindings.moveArmToReefLevel4()
        .ifPresent(
            trigger -> trigger.toggleOnTrue(
                armSubsystem.run(armSubsystem::moveToLevel4)
                    .alongWith(makeSlowModeCommand())
                    .alongWith(ledSubsystem.runPatternAsCommand(solid(kBlue)))
                    .finallyDo(armSubsystem::stop)));
    controlBindings.releaseCoral()
        .ifPresent(
            trigger -> trigger.whileTrue(
                gamePieceManipulatorSubsystem.run(gamePieceManipulatorSubsystem::scoreCoral)
                    .finallyDo(gamePieceManipulatorSubsystem::stop)));

    controlBindings.parkArm()
        .ifPresent(trigger -> trigger.onTrue(armSubsystem.runOnce(armSubsystem::park).finallyDo(armSubsystem::stop)));

    controlBindings.tuneArm().ifPresent(trigger -> trigger.onTrue(new TuneArmCommand(armSubsystem)));

    controlBindings.moveArmToReefLowerAlgae()
        .ifPresent(
            trigger -> trigger.onTrue(
                armSubsystem.run(armSubsystem::moveToAlgaeLevel1)
                    .alongWith(gamePieceManipulatorSubsystem.run(gamePieceManipulatorSubsystem::intakeAlgae))
                    .alongWith(
                        ledSubsystem.runPatternAsCommand(
                            gradient(kContinuous, kAqua, kBlack).scrollAtRelativeSpeed(Percent.per(Second).of(100))
                                .reversed()))
                    .finallyDo(() -> {
                      armSubsystem.stop();
                      gamePieceManipulatorSubsystem.stop();
                    })));
    controlBindings.moveArmToReefUpperAlgae()
        .ifPresent(
            trigger -> trigger.onTrue(
                armSubsystem.run(armSubsystem::moveToAlgaeLevel2)
                    .alongWith(gamePieceManipulatorSubsystem.run(gamePieceManipulatorSubsystem::intakeAlgae))
                    .alongWith(
                        ledSubsystem.runPatternAsCommand(
                            gradient(kContinuous, kAqua, kBlack).scrollAtRelativeSpeed(Percent.per(Second).of(100))))
                    .finallyDo(() -> {
                      armSubsystem.stop();
                      gamePieceManipulatorSubsystem.stop();
                    })));

    controlBindings.holdAlgae().ifPresent(trigger -> trigger.toggleOnTrue(Commands.run(() -> {
      armSubsystem.moveToHoldAlgae();
      gamePieceManipulatorSubsystem.activeHoldAlgae();
    }, armSubsystem, gamePieceManipulatorSubsystem).finallyDo(() -> {
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

    // Coral scoring level selection
    controlBindings.selectCoralLevel1().ifPresent(trigger -> trigger.onTrue(runOnce(scoreChooser::selectLevel1)));
    controlBindings.selectCoralLevel3().ifPresent(trigger -> trigger.onTrue(runOnce(scoreChooser::selectLevel3)));
    controlBindings.selectCoralLevel4().ifPresent(trigger -> trigger.onTrue(runOnce(scoreChooser::selectLevel4)));

    // Left branch coral scoring
    Map<Integer, Command> scoreMapLeft = Map.ofEntries(
        entry(1, new EjectCoralCommand(gamePieceManipulatorSubsystem, armSubsystem, indexerSubsystem)),
          entry(3, autoCommands.scoreCoralLevel3Left(false)),
          entry(4, autoCommands.scoreCoralLevel4Left(false)));
    controlBindings.scoreCoralLeft()
        .ifPresent(trigger -> trigger.whileTrue(select(scoreMapLeft, scoreChooser::getSelectedLevel)));

    // Left branch coral scoring
    Map<Integer, Command> scoreMapRight = Map.ofEntries(
        entry(1, new EjectCoralCommand(gamePieceManipulatorSubsystem, armSubsystem, indexerSubsystem)),
          entry(3, autoCommands.scoreCoralLevel3Right(false)),
          entry(4, autoCommands.scoreCoralLevel4Right(false)));
    controlBindings.scoreCoralRight()
        .ifPresent(trigger -> trigger.whileTrue(select(scoreMapRight, scoreChooser::getSelectedLevel)));
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
    SmartDashboard.putData(
        "Run Tests",
          testMode.testCommand()
              .deadlineFor(ledSubsystem.setLEDSegmentsAsCommand(Color.kBlue, testMode.getTestResults())));
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