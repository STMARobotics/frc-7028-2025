// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction.kForward;
import static edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction.kReverse;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.PhotonVisionCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.GamePieceManipulatorSubsystem;
import frc.robot.subsystems.IndexerSubsystem;

@Logged(strategy = Logged.Strategy.OPT_IN)
public class RobotContainer {
  private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
  private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max
                                                                                    // angular velocity

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric().withDeadband(MaxSpeed * 0.1)
      .withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
  private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric()
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

  private final CommandXboxController joystick = new CommandXboxController(0);

  private final GamePieceManipulatorSubsystem gamePieceManipulatorSubsystem = new GamePieceManipulatorSubsystem();
  private final ArmSubsystem armSubsystem = new ArmSubsystem();
  private final IndexerSubsystem indexerSubsystem = new IndexerSubsystem();
  private final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
  private final DrivetrainTelemetry drivetrainTelemetry = new DrivetrainTelemetry(MaxSpeed);
  private final PhotonVisionCommand visionCommand = new PhotonVisionCommand(drivetrain::addVisionMeasurement);

  /* Path follower */
  private final SendableChooser<Command> autoChooser;

  public RobotContainer() {
    autoChooser = AutoBuilder.buildAutoChooser("Tests");
    SmartDashboard.putData("Auto Mode", autoChooser);

    configureBindings();

    visionCommand.schedule();
  }

  private void configureBindings() {
    // Note that X is defined as forward according to WPILib convention,
    // and Y is defined as to the left according to WPILib convention.
    drivetrain.setDefaultCommand(
        // Drivetrain will execute this command periodically
        drivetrain.applyRequest(
            () -> drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X
                                                                            // (left)
        ));

    joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
    joystick.b()
        .whileTrue(
            drivetrain.applyRequest(
                () -> point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))));

    joystick.pov(0).whileTrue(drivetrain.applyRequest(() -> forwardStraight.withVelocityX(0.5).withVelocityY(0)));
    joystick.pov(180).whileTrue(drivetrain.applyRequest(() -> forwardStraight.withVelocityX(-0.5).withVelocityY(0)));

    // reset the field-centric heading on left bumper press
    joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

    drivetrain.registerTelemetry(drivetrainTelemetry::telemeterize);
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
}
