package frc.robot.commands.autonomous.drivetrain;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveTrainConstants;
import frc.robot.subsystems.DriveTrain;

import java.time.Duration;
import java.time.Instant;

public class AutoBalance extends CommandBase {


    private static final double TARGET_ANGLE = 0.0;
    private final DriveTrain m_driveTrain;
    private final PIDController m_pidController;
    
    private Instant lastTime = Instant.now();
    private double lastPos;

    public AutoBalance(DriveTrain driveTrain) {
        m_driveTrain = driveTrain;

        SmartDashboard.putNumber("AutoBalance/PID_kp", 0.03);

        final double kp = SmartDashboard.getNumber("AutoBalance/PID_kp", 3.0);

        m_pidController = new PIDController(kp, 0.0, 0.0);
        m_pidController.setTolerance(0.25);

        addRequirements(driveTrain);
    }

    private double getCurrentAngle() {
        return m_driveTrain.getRoll();
    }

    @Override
    public void initialize() {
        m_driveTrain.reset();
        m_pidController.reset();
    }

    @Override
    public void execute() {
        final double currAngle = getCurrentAngle();
        lastPos = currAngle;

        double nextVal = MathUtil.clamp(m_pidController.calculate(currAngle, TARGET_ANGLE), -0.6, 0.6);
        double rotation = 0;//-m_driveTrain.getHeading();

        double speed = MathUtil.clamp(nextVal, -1.0, 1.0);

        m_driveTrain.arcadeDrive(speed, rotation);
    }

    @Override
    public boolean isFinished() {
        return m_pidController.atSetpoint();
    }

    @Override
    public void end(boolean interrupted) {
        m_driveTrain.stop();
    }
}
