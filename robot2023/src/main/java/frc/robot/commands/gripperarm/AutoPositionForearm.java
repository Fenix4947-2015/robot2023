// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.gripperarm;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.GripperArm;
import frc.robot.subsystems.GripperArm.VerticalArmPosition;

/**
 * An example command that uses an example subsystem.
 */
public class AutoPositionForearm extends CommandBase {
    private static final double DEADBAND = 0.2;

    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final GripperArm m_gripperArm;

    private static final double KP = 0.0;
    private static final double KI = 0.0;
    private static final double KD = 0.0;
    private final PIDController pid = new PIDController(KP, KI, KD);

    private final ArmPosition desiredPosition;

    public AutoPositionForearm(GripperArm gripperArm, ArmPosition desiredPosition) {
        m_gripperArm = gripperArm;
        this.desiredPosition = desiredPosition;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(gripperArm);
    }


    public enum ArmPosition {
        PICK_ELEM_FLOOR(0.0, VerticalArmPosition.REAR),
        DEPOSIT_CONE_L1(0.0, VerticalArmPosition.REAR),
        DEPOSIT_CONE_L2(0.0, VerticalArmPosition.CENTRE),
        DEPOSIT_CONE_L3(0.0, VerticalArmPosition.FORWARD),
        DEPOSIT_CUBE_L1(0.0, VerticalArmPosition.REAR),
        DEPOSIT_CUBE_L2(0.0, VerticalArmPosition.REAR),
        DEPOSIT_CUBE_L3(0.0, VerticalArmPosition.REAR)
        ;
        private ArmPosition(double encoderAngle, VerticalArmPosition verticalArmPosition) {
            this.encoderAngle = encoderAngle;
            this.verticalArmPosition = verticalArmPosition;
        }
        private final double encoderAngle;
        private final VerticalArmPosition verticalArmPosition;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    private static final double CLAMP_PID_SPEED = 0.8;
    @Override
    public void execute() {
        double output = pid.calculate(m_gripperArm.getEncoderDistance(), desiredPosition.encoderAngle);
        double speed = MathUtil.clamp(output, -CLAMP_PID_SPEED, CLAMP_PID_SPEED);
        m_gripperArm.moveForearm(speed);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return pid.atSetpoint();
    }
}
