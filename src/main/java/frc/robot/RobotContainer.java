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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.ArmCommand;
import frc.robot.commands.ArmMoveCommand;
import frc.robot.commands.ElevatorMoveCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.ShiftCommand;
import frc.robot.commands.ZeroArm;
import frc.robot.commands.Auto.AutoCubeTaxi;
import frc.robot.commands.Auto.AutoCubeTaxiBalance;
import frc.robot.commands.Auto.Auto3PieceSmooth;
import frc.robot.commands.Auto.AutoBalanceSwerve;
import frc.robot.commands.Auto.AutoCubeTaxi;
import frc.robot.commands.Auto.TestPath;
import frc.robot.commands.Auto.AutoBalanceSwerve.RobotDirectionToStation;
import frc.robot.commands.Auto.AutoCubeTaxi.Side;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final Elevator m_Elevator = new Elevator(31, 30);
  private final Arm m_Arm = new Arm(Constants.ArmConstants.kArmTalonSRX);
  private final Intake m_Intake = new Intake(Constants.ArmConstants.kIntakeSparkmax);

  // The driver's controller
  CommandPS4Controller m_driverController = new CommandPS4Controller(OIConstants.kDriverControllerPort);
  CommandPS4Controller m_AuxController = new CommandPS4Controller(OIConstants.kAuxControllerPort);

  // Aux Controller Triggers
  final Trigger auxR1 = m_AuxController.R1(); // Cube
  final Trigger auxR2 = m_AuxController.R2(); // Cone
  final Trigger auxL2 = m_AuxController.L2(); // Elevator Down
  final Trigger auxSquare = m_AuxController.square(); // Cone Outtake
  final Trigger auxTriangle = m_AuxController.triangle(); // Cone Intake
  final Trigger auxCross = m_AuxController.cross(); // Cone Shootttt
  final Trigger auxDPADU = m_AuxController.povUp(); // Elevator High
  final Trigger auxDPADD = m_AuxController.povDown(); // Ground Pickup/Hybrid
  final Trigger auxDPADR = m_AuxController.povRight(); // Hybrid Cone
  final Trigger auxDPADL = m_AuxController.povLeft(); // Mid Cone
  final Trigger auxCircle = m_AuxController.circle(); // Hybrid Cube

  // Driver Controller Triggers
  final Trigger driveCross = m_driverController.cross();
  final Trigger driveDPADD = m_driverController.povDown();
  final Trigger driveR2 = m_driverController.R2();

  boolean controlShifted = false;

  final HashMap<String, Command> eventMap = new HashMap<>(){{
    put("ground_intake", 
      Commands.sequence(
        m_Arm.moveCommand(ArmConstants.kGroundIntake),
        m_Intake.intakeCommand(25, 1)
      )
    );

    put("intake_hold",
      Commands.sequence(
        m_Intake.intakeCommand(5, ArmConstants.kIntakeHold)
      )
    );

    put("cube_mid",
      Commands.sequence(
        m_Elevator.moveCommand(ElevatorConstants.kCubeMid),
        m_Arm.moveCommand(ArmConstants.kCubeMid),
        Commands.waitSeconds(1),
        m_Intake.intakeCommand(25, ArmConstants.kIntake),
        Commands.waitSeconds(0.25),
        m_Intake.intakeCommand(0, 0),
        m_Elevator.moveCommand(0),
        m_Arm.moveCommand(0)
      )
    );

    put("auto_balance_away", new AutoBalanceSwerve(m_robotDrive, RobotDirectionToStation.AWAY));

    put("auto_balance_toward", new AutoBalanceSwerve(m_robotDrive, RobotDirectionToStation.TOWARD));

    put("cone_outtake",
      Commands.sequence(
        m_Intake.intakeCommand(5, ArmConstants.kIntakeHold),
        Commands.waitSeconds(1),
        m_Elevator.moveCommand(0)
      )
    );

    put("cone_mid",
      Commands.sequence(
        m_Elevator.moveCommand(ElevatorConstants.kConeMid),
        m_Arm.moveCommand(ArmConstants.kConeMid),
        Commands.waitSeconds(1),
        m_Intake.intakeCommand(25, ArmConstants.kOuttake),
        Commands.waitSeconds(1),
        m_Intake.intakeCommand(0, 0),
        m_Elevator.moveCommand(0),
        m_Arm.moveCommand(0)
      )
    );

    put("move_cone_mid",
      Commands.sequence(
        m_Elevator.moveCommand(ElevatorConstants.kConeMid),
        m_Arm.moveCommand(ArmConstants.kConeMid)
      )
    );

    put("cone_high",
      Commands.sequence(
        m_Elevator.moveCommand(ElevatorConstants.kElevatorMax),
        m_Arm.moveCommand(ArmConstants.kArmMax),
        Commands.waitSeconds(2),
        m_Intake.intakeCommand(25, ArmConstants.kOuttake),
        Commands.waitSeconds(1),
        m_Intake.intakeCommand(0, 0),
        m_Elevator.moveCommand(0),
        m_Arm.moveCommand(0)
      )
    );
  }};

  final SwerveAutoBuilder m_autoBuilder = new SwerveAutoBuilder(
      m_robotDrive::getPose,
      m_robotDrive::resetOdometry,
      DriveConstants.kDriveKinematics,
      new PIDConstants(5.0, 0.0, 0.0),
      new PIDConstants(0.5, 0, 0.9),
      m_robotDrive::setModuleStates,
      eventMap,
      true,
      m_robotDrive);

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
                MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband),
                true, true),
            m_robotDrive));
    m_robotDrive.zeroHeading();
  }

  public boolean getIsControlShifted() {
    return controlShifted;
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
  private void configureButtonBindings() { // controls
    // Zero swerve
    driveCross.onTrue(new InstantCommand(() -> m_robotDrive.zeroHeading(), m_robotDrive));
    driveDPADD.onTrue(new ZeroArm(m_Arm));
    driveR2.onTrue(new InstantCommand(() -> m_robotDrive.setX()));

    auxDPADD.onTrue(new ArmMoveCommand(m_Arm, Constants.ArmConstants.kGroundIntake)); // Ground Pickup
    auxL2.onTrue(new ElevatorMoveCommand(m_Elevator, 0) // Elevator Down
      .andThen(new WaitCommand(0.5))
      .alongWith(new ArmMoveCommand(m_Arm, 0)));

    auxTriangle.onTrue(new IntakeCommand(m_Intake, 25, Constants.ArmConstants.kIntake)).onFalse(new IntakeCommand(m_Intake, 5, Constants.ArmConstants.kIntakeHold)); // Cone Intake
    auxSquare.onTrue(new IntakeCommand(m_Intake, 25, Constants.ArmConstants.kOuttake)).onFalse(new IntakeCommand(m_Intake, 0, 0)); // Cone Outtake
    auxCross.onTrue(new IntakeCommand(m_Intake, 25, -1)).onFalse(new IntakeCommand(m_Intake, 0, 0));
    auxCircle.onTrue(new ArmMoveCommand(m_Arm, Constants.ArmConstants.kCubeMid));

    auxDPADU.onTrue(new ElevatorMoveCommand(m_Elevator, Constants.ElevatorConstants.kElevatorMax).alongWith(new ArmMoveCommand(m_Arm, Constants.ArmConstants.kArmMax))); // 
    auxDPADR.onTrue(new ArmMoveCommand(m_Arm, ArmConstants.kHybridCone));
    auxDPADL.onTrue(new ElevatorMoveCommand(m_Elevator, Constants.ElevatorConstants.kConeMid).alongWith(new ArmMoveCommand(m_Arm, Constants.ArmConstants.kConeMid))
    );

    auxR1.onTrue(new ElevatorMoveCommand(m_Elevator, Constants.ElevatorConstants.kConeMid).alongWith(new ArmMoveCommand(m_Arm, Constants.ArmConstants.kConeMid))
    ); // Cone Mid
    auxR2.onTrue(new ElevatorMoveCommand(m_Elevator, Constants.ElevatorConstants.kConeDoubleSub).alongWith(new ArmMoveCommand(m_Arm, Constants.ArmConstants.kArmMax))); // Cube Mid
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand(String selectedAuto) {
    switch (selectedAuto) {
      case "Auto Cube Taxi Smooth":
        return new AutoCubeTaxi(m_autoBuilder, Side.SMOOTH);
      case "Auto Cube Taxi Bump":
        return new AutoCubeTaxi(m_autoBuilder, Side.BUMP);
      case "Auto Cube Balance":
        return Commands.sequence(
          m_Elevator.moveCommand(ElevatorConstants.kCubeMid),
          m_Arm.moveCommand(ArmConstants.kCubeMid),
          Commands.waitSeconds(1),
          m_Intake.intakeCommand(25, ArmConstants.kIntake),
          Commands.waitSeconds(0.25),
          m_Intake.intakeCommand(0, 0),
          m_Elevator.moveCommand(0),
          m_Arm.moveCommand(0),
          new AutoBalanceSwerve(m_robotDrive, RobotDirectionToStation.AWAY)
        );
      case "Auto Cube Taxi Balance":
        return new AutoCubeTaxiBalance(m_autoBuilder);
      case "Test Path":
        return new TestPath(m_autoBuilder);
      case "Balance Test":
        return new AutoBalanceSwerve(m_robotDrive, RobotDirectionToStation.TOWARD);
      case "Auto Cube + 2 Piece":
        return m_autoBuilder.fullAuto(PathPlanner.loadPath("Cube + 2 Piece Smooth Vit Red", new PathConstraints(1.5, 3)));
      default:
        return new AutoCubeTaxi(m_autoBuilder, Side.SMOOTH);
    }
  }
}
