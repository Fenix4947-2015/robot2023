// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.gripperarm;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.GripperArm;

/**
 * An example command that uses an example subsystem.
 */
public class MoveForearm extends CommandBase {
    private static final double DEADBAND = 0.2;
    private static final double KEEP_STATIONARY_THRESHOLD_ANGLE = 24.0;
    private static final double KEEP_STATIONARY_POWER = 0.1;

    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final GripperArm m_gripperArm;
    private final XboxController m_controller;

    /**
     * Creates a new ExampleCommand.
     *
     * @param subsystem The subsystem used by this command.
     */
    public MoveForearm(GripperArm gripperArm, XboxController xboxController) {
        m_gripperArm = gripperArm;
        m_controller = xboxController;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(gripperArm);
    }

    private static double applyDeadband(double val) {
        return Math.abs(val) < DEADBAND ? 0.0 : val;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double speed = applyDeadband(-m_controller.getRightY());

        if (speed == 0.0 && m_gripperArm.getCurrentVerticalArmPosition() == GripperArm.VerticalArmPosition.FORWARD
                && m_gripperArm.getEncoderDistance() >= KEEP_STATIONARY_THRESHOLD_ANGLE) {
            speed = KEEP_STATIONARY_POWER;
        }

        SmartDashboard.putNumber("MoveForearm/speed", speed);
        m_gripperArm.moveForearm(speed);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
