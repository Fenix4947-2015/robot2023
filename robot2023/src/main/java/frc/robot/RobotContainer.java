// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.StartMotor;
import frc.robot.commands.StopAll;
import frc.robot.commands.StopArm;
import frc.robot.commands.StopMotor;
import frc.robot.commands.drivetrain.DriveArcade;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.GripperArm;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);
  private final CommandXboxController m_helperController =
      new CommandXboxController(OperatorConstants.kHelperControllerPort);

  // The robot's subsystems and commands are defined here...

  // Subsystems.
  private final DriveTrain m_driveTrain = new DriveTrain();
  private final GripperArm m_gripperArm = new GripperArm();
  
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

  // Commands.
  private final DriveArcade m_driveArcade = new DriveArcade(m_driverController.getHID(), m_driveTrain);
  private final CommandBase m_shiftHigh = new InstantCommand(m_driveTrain::shiftHigh);
  private final CommandBase m_shiftLow = new InstantCommand(m_driveTrain::shiftLow);
  private final CommandBase m_stopAll = new StopAll(m_driveTrain);
  private final StopArm m_stopArm = new StopArm(m_gripperArm);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    configureDefaultCommands();
    configureAutonomousCommands();
    configureSmartDashboard();
  }

  private void configureDefaultCommands() {
    m_driveTrain.setDefaultCommand(m_driveArcade);
    m_gripperArm.setDefaultCommand(m_stopArm);
  }

  private void configureAutonomousCommands() {
    // m_autonomousDelayChooser.setDefaultOption("0", 0);
    // for (int i = 1; i <= 15; ++i) {
    //     m_autonomousDelayChooser.addOption(String.valueOf(i), i);
    // }

    // m_autonomousCommandChooser.setDefaultOption("None", m_autoNone);
    // m_autonomousCommandChooser.addOption("Exit Tarmac (PID)", m_autoExitTarmacPID);
    // //m_autonomousCommandChooser.addOption("Exit Tarmac (timer)", m_autoExitTarmacTimer);
    // m_autonomousCommandChooser.addOption("Shoot 1 Ball", m_autoShoot1Ball);
    // m_autonomousCommandChooser.addOption("Shoot 2 Balls 155deg", m_autoShoot2Balls155);
    // m_autonomousCommandChooser.addOption("Shoot 2 Balls 180deg", m_autoShoot2Balls180);

    // SmartDashboard.putData("Autonomous Delay", m_autonomousDelayChooser);
    // SmartDashboard.putData("Autonomous Command", m_autonomousCommandChooser);
}

private void configureSmartDashboard() {
    SmartDashboard.putData(CommandScheduler.getInstance());

    SmartDashboard.putData(m_driveTrain);
    // SmartDashboard.putData(m_intake);
    // SmartDashboard.putData(m_shooter);

    SmartDashboard.putData("Commands/Shift High", m_shiftHigh);
    SmartDashboard.putData("Commands/Shift Low", m_shiftLow);        

    // SmartDashboard.putData("Commands/Move Shooter Up", m_moveShooterUp);
    // SmartDashboard.putData("Commands/Move Shooter Down", m_moveShooterDown);
    // SmartDashboard.putData("Commands/Shoot", m_shootNear);
    // SmartDashboard.putData("Commands/Spin Shooter", m_spinShooter);
    // SmartDashboard.putData("Commands/Stop Shooter", m_stopShooter);

    // SmartDashboard.putData("Commands/Latch Intake", m_latchIntake);
    // SmartDashboard.putData("Commands/Unlatch Intake", m_unlatchIntake);

    // SmartDashboard.putData("Commands/Unlatch Winch", m_unlatchWinch);

    SmartDashboard.putData("Commands/Stop All", m_stopAll);

    // SmartDashboard.putBoolean("Winch/Override 30 sec endmatch", false);
}

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    // new Trigger(m_exampleSubsystem::exampleCondition)
    //     .onTrue(new ExampleCommand(m_exampleSubsystem));

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    //m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());
    //m_driverController.a().whileTrue(m_startMotor);

    m_driverController.a().onTrue(m_shiftHigh);
    m_driverController.x().onTrue(m_shiftLow);
    m_driverController.b().whileTrue(new InstantCommand(m_gripperArm::downForeArm));
    m_driverController.y().whileTrue(new InstantCommand(m_gripperArm::upForearm));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return Autos.exampleAuto(m_exampleSubsystem);
  }
}
