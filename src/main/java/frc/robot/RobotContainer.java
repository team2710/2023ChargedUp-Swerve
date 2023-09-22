// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.ArmMoveCommand;
import frc.robot.commands.ElevatorMoveCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.ZeroArm;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import java.util.List;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final Elevator m_Elevator = new Elevator(Constants.ElevatorConstants.kLeftSparkmax, Constants.ElevatorConstants.kRightSparkmax);
  private final Arm m_Arm = new Arm(Constants.ArmConstants.kArmTalonSRX);
  private final Intake m_Intake = new Intake(Constants.ArmConstants.kIntakeSparkmax);

  // The driver's controller
  CommandPS4Controller m_driverController = new CommandPS4Controller(OIConstants.kDriverControllerPort);
  CommandPS4Controller m_AuxController = new CommandPS4Controller(OIConstants.kAuxControllerPort);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> m_robotDrive.drive(
                -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband),
                true, true),
            m_robotDrive));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
    // new JoystickButton(m_driverController, Button.kR1.value)
    //     .whileTrue(new RunCommand(
    //         () -> m_robotDrive.setX(),
    //         m_robotDrive));

    Trigger rightBumper = m_AuxController.R1();
    Trigger leftBumper = m_AuxController.L1();
    rightBumper.onTrue(new ElevatorMoveCommand(m_Elevator, Constants.ElevatorConstants.kElevatorMax)
            // .alongWith(new DriveCommand(drivetrain, driverController, 2))
            .alongWith(new ArmMoveCommand(m_Arm, 5250)));
        leftBumper.onTrue(new ElevatorMoveCommand(m_Elevator, 0)
            .andThen(new WaitCommand(0.5))
            .alongWith(new ArmMoveCommand(m_Arm, 0)));

    Trigger crossButton = m_AuxController.cross();
    crossButton.onTrue(new ArmMoveCommand(m_Arm, 0));

    Trigger driverPovDown = m_driverController.povDown();
    driverPovDown.onTrue(new ZeroArm(m_Arm));

    Trigger yButton = m_AuxController.triangle();
    Trigger xButton = m_AuxController.square();
    Trigger povDownButton = m_AuxController.povDown();
    yButton.onTrue(new IntakeCommand(m_Intake, 25, Constants.ArmConstants.kIntake))
        .onFalse(new IntakeCommand(m_Intake, 5, Constants.ArmConstants.kIntakeHold));
    xButton.onTrue(new IntakeCommand(m_Intake, 25, Constants.ArmConstants.KOuttake * -1))
        .onFalse(new IntakeCommand(m_Intake, 25, 0));
    povDownButton.onTrue(new IntakeCommand(m_Intake, 25, 0));

    Trigger rightButtonTrigger = m_AuxController.povLeft();
    rightButtonTrigger.onTrue(new ArmMoveCommand(m_Arm, 2000));

    Trigger povRightButton = m_AuxController.povRight();
    povRightButton.onTrue(new ElevatorMoveCommand(m_Elevator, Constants.ElevatorConstants.kCubeMid)
            // .alongWith(new DriveCommand(drivetrain, driverController, 2))
        .alongWith(new ArmMoveCommand(m_Arm, 4500)));

        // Trigger bButton = auxController.b();
    Trigger circleButton = m_AuxController.circle();
    circleButton.onTrue(new ElevatorMoveCommand(m_Elevator, Constants.ElevatorConstants.kCubeMid+1)
                // .alongWith(new DriveCommand(drivetrain, driverController, 2))
        .alongWith(new ArmMoveCommand(m_Arm, 5250)));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // Create config for trajectory
    TrajectoryConfig config = new TrajectoryConfig(
        AutoConstants.kMaxSpeedMetersPerSecond,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(DriveConstants.kDriveKinematics);

    // An example trajectory to follow. All units in meters.
    Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(0)),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(3, 0, new Rotation2d(0)),
        config);

    var thetaController = new ProfiledPIDController(
        AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
        exampleTrajectory,
        m_robotDrive::getPose, // Functional interface to feed supplier
        DriveConstants.kDriveKinematics,

        // Position controllers
        new PIDController(AutoConstants.kPXController, 0, 0),
        new PIDController(AutoConstants.kPYController, 0, 0),
        thetaController,
        m_robotDrive::setModuleStates,
        m_robotDrive);

    // Reset odometry to the starting pose of the trajectory.
    m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose());

    // Run path following command, then stop at the end.
    return swerveControllerCommand.andThen(() -> m_robotDrive.drive(0, 0, 0, false, false));
  }
}