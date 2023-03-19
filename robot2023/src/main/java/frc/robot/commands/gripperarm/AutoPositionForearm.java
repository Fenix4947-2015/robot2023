// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.gripperarm;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.GripperArm;
import frc.robot.subsystems.GripperArm.VerticalArmPosition;

/**
 * An example command that uses an example subsystem.
 */
public class AutoPositionForearm extends CommandBase {
    private static final double DEADBAND = 0.2;
    private static final double KP = 0.5;
    private static final double KI = 0.0;
    private static final double KD = 0.0;
    private static final double CLAMP_PID_SPEED = 0.8;
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final GripperArm m_gripperArm;
    private final XboxController m_controller;
    private final PIDController pid = new PIDController(KP, KI, KD);
    private double _targetPosition;


    public AutoPositionForearm(GripperArm gripperArm, XboxController xboxController) {
        m_gripperArm = gripperArm;
        m_controller = xboxController;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(gripperArm);
        pid.setTolerance(1.0);
        _targetPosition = m_gripperArm.getEncoderDistance();
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    private static double applyDeadband(double val) {
        return Math.abs(val) < DEADBAND ? 0.0 : val;
    }

    @Override
    public void execute() {
        double move = applyDeadband(-m_controller.getRightY());
        double currentPosition = m_gripperArm.getEncoderDistance();
        double speed = move;
        if (m_gripperArm.isAutoEnabled()) {
            _targetPosition = move == 0 ? _targetPosition : currentPosition + move;
            speed = calculatePidMovement(_targetPosition);
        }

        SmartDashboard.putNumber("AutoPosForearm/currentPosition", currentPosition);
        SmartDashboard.putNumber("AutoPosForearm/targetPosition", _targetPosition);
        SmartDashboard.putNumber("AutoPosForearm/speed", speed);
        SmartDashboard.putNumber("AutoPosForearm/setpoint", pid.getSetpoint());
        SmartDashboard.putBoolean("AutoPosForearm/isAtSetpoint", pid.atSetpoint());

        m_gripperArm.moveForearm(speed);
    }

    private double calculatePidMovement(double desiredEncoderAngle) {
        pid.setSetpoint(desiredEncoderAngle);
        double output = pid.calculate(m_gripperArm.getEncoderDistance(), desiredEncoderAngle);
        double speed = MathUtil.clamp(output, -CLAMP_PID_SPEED, CLAMP_PID_SPEED);
        return speed;
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
