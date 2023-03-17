// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.StopAll;
import frc.robot.commands.autonomous.AutoAim;
import frc.robot.commands.drivetrain.DriveArcade;
import frc.robot.commands.gripperarm.AutoPositionForearm;
import frc.robot.commands.gripperarm.HomeForearm;
import frc.robot.commands.gripperarm.MoveForearm;
import frc.robot.commands.gripperarm.StopArm;
import frc.robot.limelight.Limelight;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.GripperArm;

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
    private final SmartDashboardSettings m_smartDashboardSettings = new SmartDashboardSettings();
    private final Limelight m_limelight = new Limelight();

    // Commands.
    private final AutoAim _autoAimPick = new AutoAim(AutoAim.AUTOAIM_PICK_PIPELINE, m_driveTrain, m_limelight,
    m_smartDashboardSettings);
    private final AutoAim _autoAimPlace = new AutoAim(AutoAim.AUTOAIM_PLACE_PIPELINE, m_driveTrain, m_limelight,
    m_smartDashboardSettings);


    private final DriveArcade m_driveArcade = new DriveArcade(m_driverController.getHID(), m_driveTrain);
    private final CommandBase m_shiftHigh = new InstantCommand(m_driveTrain::shiftHigh);
    private final CommandBase m_shiftLow = new InstantCommand(m_driveTrain::shiftLow);
    private final CommandBase m_stopAll = new StopAll(m_driveTrain);
    private final StopArm m_stopArm = new StopArm(m_gripperArm);
    private final MoveForearm m_moveForearm = new MoveForearm(m_gripperArm, m_driverController.getHID());
    private final HomeForearm m_homeForearm = new HomeForearm(m_gripperArm);

    private final CommandBase m_extendKickstand = new InstantCommand(m_gripperArm::extendKickstand);
    private final CommandBase m_retractKickstand = new InstantCommand(m_gripperArm::retractKickstand);
    private final CommandBase m_lockElbow = new InstantCommand(m_gripperArm::lockElbow);
    private final CommandBase m_unlockElbow = new InstantCommand(m_gripperArm::unlockElbow);
    private final CommandBase m_closeGripper = new InstantCommand(m_gripperArm::closeGripper);
    private final CommandBase m_openGripper = new InstantCommand(m_gripperArm::openGripper);

    private final AutoPositionForearm m_positionArmHome = new AutoPositionForearm(m_gripperArm, AutoPositionForearm.ArmPosition.HOME);
    private final AutoPositionForearm m_positionArmPickElemFloor = new AutoPositionForearm(m_gripperArm, AutoPositionForearm.ArmPosition.PICK_ELEM_FLOOR);
    private final AutoPositionForearm m_positionArmPickElemStation = new AutoPositionForearm(m_gripperArm, AutoPositionForearm.ArmPosition.PICK_ELEM_STATION);
    private final AutoPositionForearm m_positionArmPlaceElemMid = new AutoPositionForearm(m_gripperArm, AutoPositionForearm.ArmPosition.PLACE_ELEM_MID);
    private final AutoPositionForearm m_positionArmPlaceElemTop = new AutoPositionForearm(m_gripperArm, AutoPositionForearm.ArmPosition.PLACE_ELEM_TOP);

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // Configure the trigger bindings
        configureBindings();
        configureDefaultCommands();
        configureAutonomousCommands();
        configureSmartDashboard();
    }

    private void configureDefaultCommands() {
        m_driveTrain.setDefaultCommand(m_driveArcade);
        m_gripperArm.setDefaultCommand(m_moveForearm);
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

        m_driverController.rightBumper().onTrue(m_shiftHigh);
        m_driverController.leftBumper().onTrue(m_shiftLow);
        m_driverController.b().whileTrue(m_closeGripper);
        m_driverController.y().whileTrue(m_openGripper);
        m_driverController.a().whileTrue(_autoAimPick);
        m_driverController.povUp().onTrue(m_extendKickstand);
        m_driverController.povDown().onTrue(m_retractKickstand);
        
        m_driverController.povLeft().onTrue(new InstantCommand(m_gripperArm::moveVerticalArmBackward));
        m_driverController.povRight().onTrue(new InstantCommand(m_gripperArm::moveVerticalArmForward));

        m_helperController.back().onTrue(m_unlockElbow);
        m_helperController.start().onTrue(m_lockElbow);

        m_helperController.a().whileTrue(m_positionArmHome);
        m_helperController.b().whileTrue(m_positionArmPickElemFloor);
        m_helperController.x().whileTrue(m_positionArmPickElemStation);
        m_helperController.y().whileTrue(m_positionArmPlaceElemMid);
        m_helperController.rightBumper().whileTrue(m_positionArmPlaceElemTop);
    }

    public void teleopInit() {
        m_gripperArm.initTeleop();
        // m_homeForearm.schedule();;
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An example command will be run in autonomous
        //return Autos.exampleAuto(m_exampleSubsystem);
        return m_stopAll;
    }
}
