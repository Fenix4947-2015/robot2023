// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.gripperarm;

import org.ejml.dense.block.MatrixOps_DDRB;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.exchange.SubSystemDataExchange;
import frc.robot.subsystems.GripperArm;

/**
 * An example command that uses an example subsystem.
 */
public class AutoPositionForearm extends CommandBase {
    private static final double DEADBAND = 0.2;
    private static final double KP = 0.5;
    // private static final double KI = 0.0;
    // private static final double KD = 0.0;
    private static final double CLAMP_PID_SPEED = 0.8;
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final GripperArm m_gripperArm;
    private final XboxController m_controller;
    private double _targetPosition;
    private boolean _isMoving;
    private boolean _autoPositionEnabled;
    private SubSystemDataExchange m_dataExchange;


    public AutoPositionForearm(GripperArm gripperArm, XboxController xboxController, SubSystemDataExchange dataExchange) {
        m_gripperArm = gripperArm;
        m_controller = xboxController;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(gripperArm);
        _targetPosition = m_gripperArm.getEncoderDistance();
        _isMoving = false;
        _autoPositionEnabled = true;
        m_dataExchange = dataExchange;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        _targetPosition = getCurrentForearmPosition();
    }

    private static double applyDeadband(double val) {
        return Math.abs(val) < DEADBAND ? 0.0 : val;
    }

    private double getCurrentForearmPosition() {
        return m_gripperArm.getEncoderDistance();
    }

    private double clampTargetValue(double value) {
        if (value < m_gripperArm.getCurrentVerticalArmPosition().homedAngleEncoderValue) {
            return m_gripperArm.getCurrentVerticalArmPosition().homedAngleEncoderValue;
        } else if (value > m_gripperArm.getCurrentVerticalArmPosition().maxAngleEncoderValue) {
            return m_gripperArm.getCurrentVerticalArmPosition().maxAngleEncoderValue;
        }
        return value;
    }

    @Override
    public void execute() {
        double move = applyDeadband(-m_controller.getRightY());
        double currentPosition = getCurrentForearmPosition();
        double speed = move;
        if (move > 0.0) {
            _autoPositionEnabled = true;
        } else if (move < 0.0) {
            _autoPositionEnabled = false;
        }
        if (m_gripperArm.isAutoEnabled() && _autoPositionEnabled) {
            if (move != 0) {
                double moveFactor = currentPosition < (m_gripperArm.getCurrentVerticalArmPosition().maxAngleEncoderValue - 2.0) ? 2.0 : 1.0;
                _targetPosition = currentPosition + move * moveFactor;
                _isMoving = true;
            } else if (_isMoving) {
                _isMoving = false;
                _targetPosition = currentPosition;
            }

            _targetPosition = clampTargetValue(_targetPosition);

            if (_targetPosition >= currentPosition) {
                speed = calculatePidMovement(_targetPosition);
            }
        }

        if (currentPosition > m_gripperArm.getCurrentVerticalArmPosition().autoAngleEncoderValue) {
            m_dataExchange.setSpeedLimited(true);
        } else {
            m_dataExchange.setSpeedLimited(false);
        }

        //SmartDashboard.putNumber("AutoPosForearm/currentPosition", currentPosition);
        //SmartDashboard.putNumber("AutoPosForearm/targetPosition", _targetPosition);
        //SmartDashboard.putNumber("AutoPosForearm/speed", speed);
        m_gripperArm.moveForearm(speed);
    }

    private double calculatePidMovement(double desiredEncoderAngle) {
        double output = (desiredEncoderAngle - m_gripperArm.getEncoderDistance()) * KP;
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
