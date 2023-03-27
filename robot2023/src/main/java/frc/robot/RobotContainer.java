// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.InstantCommands;
import frc.robot.commands.StopAll;
import frc.robot.commands.autonomous.AutoAim;
import frc.robot.commands.autonomous.DepositConeHigh;
import frc.robot.commands.autonomous.DepositConeMiddle;
import frc.robot.commands.drivetrain.DriveArcade;
import frc.robot.commands.gripperarm.*;
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


    private final DriveArcade m_driveArcade = new DriveArcade(m_driverController.getHID(), m_driveTrain);
    private final CommandBase m_shiftHigh = new InstantCommand(m_driveTrain::shiftHigh);
    private final CommandBase m_shiftLow = new InstantCommand(m_driveTrain::shiftLow);
    private final CommandBase m_stopAll = new StopAll(m_driveTrain);
    //private final MoveForearm m_moveForearm = new MoveForearm(m_gripperArm, m_driverController.getHID());
    private final AutoPositionForearm m_autoPositionForearm = new AutoPositionForearm(m_gripperArm, m_helperController.getHID());

    private final CommandBase m_toggleElbow = new InstantCommand(m_gripperArm::toggleElbow);

    private final InstantCommands m_instantCommands = new InstantCommands(m_gripperArm);

    private final AutoPositionArm m_positionArmHome = new AutoPositionArm(m_gripperArm, AutoPositionArm.ArmPosition.HOME);
    private final AutoPositionArm m_positionArmPickElemFloor = new AutoPositionArm(m_gripperArm, AutoPositionArm.ArmPosition.PICK_ELEM_FLOOR);
    private final AutoPositionArm m_positionArmPickElemStation = new AutoPositionArm(m_gripperArm, AutoPositionArm.ArmPosition.PICK_ELEM_STATION);
    private final AutoPositionArm m_positionArmPlaceElemMid = new AutoPositionArm(m_gripperArm, AutoPositionArm.ArmPosition.PLACE_ELEM_MID);
    private final AutoPositionArm m_positionArmPlaceElemTop = new AutoPositionArm(m_gripperArm, AutoPositionArm.ArmPosition.PLACE_ELEM_TOP);


    // Autonomous commands.
    private final CommandBase m_autoNone = new PrintCommand("No autonomous command selected");
    private final CommandBase m_depositConeHigh = new DepositConeHigh(m_driveTrain, m_gripperArm);
    private final CommandBase m_depositConeMiddle = new DepositConeMiddle(m_driveTrain, m_gripperArm);

    private final SendableChooser<Integer> m_autonomousDelayChooser = new SendableChooser<>();
    private final SendableChooser<Command> m_autonomousCommandChooser = new SendableChooser<>();


    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // Configure the trigger bindings
        configureBindings();
        configureDefaultCommands();
        configureAutonomousCommands();
        configureSmartDashboard();

        m_gripperArm.initialize();
    }

    private void configureDefaultCommands() {
        m_driveTrain.setDefaultCommand(m_driveArcade);
        m_gripperArm.setDefaultCommand(m_autoPositionForearm);
        //m_gripperArm.setDefaultCommand(m_moveForearm);
    }

    private void configureAutonomousCommands() {
        m_autonomousDelayChooser.setDefaultOption("0", 0);
        for (int i = 1; i <= 15; ++i) {
            m_autonomousDelayChooser.addOption(String.valueOf(i), i);
        }

        m_autonomousCommandChooser.setDefaultOption("None", m_autoNone);
        m_autonomousCommandChooser.addOption("Deposit Cone Middle", m_depositConeMiddle);
        m_autonomousCommandChooser.addOption("Deposit Cone High", m_depositConeHigh);
    }

    private void configureSmartDashboard() {
        SmartDashboard.putData(CommandScheduler.getInstance());

        SmartDashboard.putData(m_driveTrain);
        SmartDashboard.putData(m_gripperArm);
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

        SmartDashboard.putData("Autonomous Delay", m_autonomousDelayChooser);
        SmartDashboard.putData("Autonomous Command", m_autonomousCommandChooser);
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
        m_driverController.b().whileTrue(m_instantCommands.closeGripper());
        m_driverController.y().whileTrue(m_instantCommands.openGripper());
        m_driverController.a().whileTrue(_autoAimPick);
        //m_driverController.back().onTrue(m_toggleElbow);
        //m_driverController.povLeft().onTrue(new InstantCommand(m_gripperArm::moveVerticalArmBackward));
        //m_driverController.povRight().onTrue(new InstantCommand(m_gripperArm::moveVerticalArmForward));
        

        m_driverController.povUp().onTrue(m_instantCommands.extendKickstand());
        m_driverController.povDown().onTrue(m_instantCommands.retractKickstand());

        m_helperController.povUp().onTrue(m_instantCommands.extendKickstand());
        m_helperController.povDown().onTrue(m_instantCommands.retractKickstand());
        m_helperController.povLeft().onTrue(new InstantCommand(m_gripperArm::moveVerticalArmBackward));
        m_helperController.povRight().onTrue(new InstantCommand(m_gripperArm::moveVerticalArmForward));

        m_helperController.back().onTrue(m_toggleElbow);
        //m_helperController.start().onTrue(m_lockElbow);

        m_helperController.start().whileTrue(m_positionArmHome);
        m_helperController.a().whileTrue(m_positionArmPickElemFloor);
        m_helperController.b().whileTrue(m_positionArmPickElemStation);
        m_helperController.x().whileTrue(m_positionArmPlaceElemMid);
        m_helperController.y().whileTrue(m_positionArmPlaceElemTop);

        m_helperController.rightBumper().onTrue(m_instantCommands.closeGripper());
        m_helperController.leftBumper().onTrue(m_instantCommands.openGripper());
    }

    public void teleopInit() {
        // m_homeForearm.schedule();
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        int autonomousDelay = getAutonomousDelay();
        if (autonomousDelay > 0) {
            return m_autonomousCommandChooser.getSelected().beforeStarting(new WaitCommand(autonomousDelay));
        }

        return m_autonomousCommandChooser.getSelected();
    }

    private int getAutonomousDelay() {
        Integer autonomousDelay = m_autonomousDelayChooser.getSelected();
        return (autonomousDelay != null) ? autonomousDelay.intValue() : 0;
    }
}
