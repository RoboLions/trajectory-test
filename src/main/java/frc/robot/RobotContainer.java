// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.CentripetalAccelerationConstraint;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import java.util.List;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  public final static DriveSubsystem driveSubsystem = new DriveSubsystem();

  private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);

  public static PIDController leftController;
  public static PIDController rightController;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {}

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // Create a voltage constraint to ensure we don't accelerate too fast
    var autoVoltageConstraint =
        new DifferentialDriveVoltageConstraint(
            new SimpleMotorFeedforward(
                DriveConstants.ksVolts,
                DriveConstants.kvVoltSecondsPerMeter,
                DriveConstants.kaVoltSecondsSquaredPerMeter),
            DriveConstants.kDriveKinematics,
            10);

    // Create config for trajectory
    TrajectoryConfig config =
        new TrajectoryConfig(
                AutoConstants.kMaxSpeedMetersPerSecond,
                AutoConstants.kMaxAccelerationMetersPerSecondSquared)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(DriveConstants.kDriveKinematics)
            // Apply the voltage constraint
            .addConstraint(autoVoltageConstraint)
            .addConstraint(new CentripetalAccelerationConstraint(AutoConstants.kMaxCentripetalAcceleration));

    // An example trajectory to follow.  All units in meters.
    Trajectory exampleTrajectory =
        TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(0, 0, new Rotation2d(0)),
            // Pass through these two interior waypoints, making an 's' curve path
            List.of(new Translation2d(1, 0.5)),
            //List.of(new Translation2d(0,0)),
            // End 3 meters straight ahead of where we started, facing forward
            new Pose2d(2, 0, new Rotation2d(0)),
            // Pass config
            config);

    RamseteController m_disabledRamsete = new RamseteController();
    m_disabledRamsete.setEnabled(false);

    /*leftController = new PIDController(0, 0, 0);
    rightController = new PIDController(0, 0, 0);*/

    // p = 7, period oscillation = .227 seconds
    leftController = new PIDController(DriveConstants.kLeftPDriveVel, DriveConstants.kLeftIDriveVel, DriveConstants.kLeftDDriveVel);
    rightController = new PIDController(DriveConstants.kRightPDriveVel, DriveConstants.kRightIDriveVel, DriveConstants.kRightDDriveVel);

    /*if (Math.abs(leftController.getSetpoint())-Math.abs(driveSubsystem.getWheelSpeeds().leftMetersPerSecond) < 0.25) {
        leftController = new PIDController(DriveConstants.kPDriveVel, 0, 0);
    } else {
        leftController = new PIDController(0, 0, 0);
    }

    if (Math.abs(rightController.getSetpoint())-Math.abs(driveSubsystem.getWheelSpeeds().rightMetersPerSecond) < 0.25) {
        rightController = new PIDController(DriveConstants.kPDriveVel, 0, 0);
    } else {
        rightController = new PIDController(0, 0, 0);
    }*/
    
    RamseteCommand ramseteCommand = new RamseteCommand(
        exampleTrajectory, // the trajectory points we put in
        driveSubsystem::getPose, // get position of bot from drive subsystem
        m_disabledRamsete, // Pass in disabledRamsete here
        //m_enabledRamsete,
        new SimpleMotorFeedforward(DriveConstants.ksVolts, DriveConstants.kvVoltSecondsPerMeter, DriveConstants.kaVoltSecondsSquaredPerMeter),
        DriveConstants.kDriveKinematics,
        driveSubsystem::getWheelSpeeds,
        leftController, // left speed PID
        rightController, // right speed PID
        // RamseteCommand passes volts to the callback
        (leftVolts, rightVolts) -> {
            driveSubsystem.tankDriveVolts(leftVolts, rightVolts);

            System.out.println("left/right speed, " + leftController.getSetpoint() + ", " + driveSubsystem.getWheelSpeeds().leftMetersPerSecond + 
            ", "+ rightController.getSetpoint() + ", " + driveSubsystem.getWheelSpeeds().rightMetersPerSecond);

            SmartDashboard.putNumber("Real left wheel speed", driveSubsystem.getWheelSpeeds().leftMetersPerSecond);
            SmartDashboard.putNumber("Setpoint left wheel speed", leftController.getSetpoint());
            SmartDashboard.putNumber("Error left wheel speed", Math.abs(leftController.getSetpoint())-Math.abs(driveSubsystem.getWheelSpeeds().leftMetersPerSecond));

            SmartDashboard.putNumber("Real right wheel speed", driveSubsystem.getWheelSpeeds().rightMetersPerSecond);
            SmartDashboard.putNumber("Setpoint right wheel speed", rightController.getSetpoint());
            SmartDashboard.putNumber("Error right wheel speed", Math.abs(rightController.getSetpoint())-Math.abs(driveSubsystem.getWheelSpeeds().rightMetersPerSecond));
        },
        driveSubsystem
    );

    /*RamseteCommand ramseteCommand = new RamseteCommand(
        exampleTrajectory, 
        driveSubsystem::getPose,
        m_disabledRamsete, 
        DriveConstants.kDriveKinematics,
        (leftVolts, rightVolts) -> {
            driveSubsystem.tankDriveVolts(leftVolts, rightVolts);
        },
        driveSubsystem
    );*/

    /*
    RamseteCommand ramseteCommand =
        new RamseteCommand(
            exampleTrajectory,
            driveSubsystem::getPose,
            new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta),
            new SimpleMotorFeedforward(
                DriveConstants.ksVolts,
                DriveConstants.kvVoltSecondsPerMeter,
                DriveConstants.kaVoltSecondsSquaredPerMeter),
            DriveConstants.kDriveKinematics,
            driveSubsystem::getWheelSpeeds,
            new PIDController(DriveConstants.kPDriveVel, 0, 0),
            new PIDController(DriveConstants.kPDriveVel, 0, 0),
            // RamseteCommand passes volts to the callback
            driveSubsystem::tankDriveVolts,
            driveSubsystem);
    */

    // Reset odometry to the starting pose of the trajectory.
    driveSubsystem.resetOdometry(exampleTrajectory.getInitialPose());

    // Run path following command, then stop at the end.
    return ramseteCommand.andThen(() -> driveSubsystem.tankDriveVolts(0, 0));
  }
}
