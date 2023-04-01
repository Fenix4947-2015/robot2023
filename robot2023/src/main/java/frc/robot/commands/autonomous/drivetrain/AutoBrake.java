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

public class AutoBrake extends CommandBase {


    private final DriveTrain m_driveTrain;
    private final PIDController m_pidController;
    
    private Instant lastTime = Instant.now();
    private double lastPos;
    private double targetPos = 0.0;

    public AutoBrake(DriveTrain driveTrain) {
        m_driveTrain = driveTrain;

        //SmartDashboard.putNumber("AutoBrake/PID_kp", 1.5);
        //SmartDashboard.putNumber("AutoBrake/PID_ki", 0.5);

        final double kp = 3.0; //SmartDashboard.getNumber("AutoBrake/PID_kp", 1.5);
        final double ki = 0.5; // SmartDashboard.getNumber("AutoBrake/PID_ki", 0.5);

        m_pidController = new PIDController(kp, ki, 0.0);
        m_pidController.setTolerance(0.02);

        addRequirements(driveTrain);
    }

    private double getCurrentPosition() {
        return m_driveTrain.getPosition();
    }

    @Override
    public void initialize() {
        //m_driveTrain.reset();
        m_pidController.reset();

        targetPos = getCurrentPosition();
    }

    @Override
    public void execute() {
        final Instant now = Instant.now();
        final double dtSeconds = ((double) Duration.between(lastTime, now).toMillis()) / 1000.0;
        final double currPos = getCurrentPosition();
        final double velocity = (currPos - lastPos) / dtSeconds;
        lastPos = currPos;

        double feedforward = DriveTrainConstants.m_feedFwdLow.calculate(velocity);

        double ffSpeed = feedforward / RobotController.getBatteryVoltage();
        double nextVal = MathUtil.clamp(m_pidController.calculate(currPos, targetPos), -0.6, 0.6);
        double rotation = 0;

        double speed = MathUtil.clamp(nextVal, -1.0, 1.0);

        //SmartDashboard.putNumber("AutoBrake/speed", speed);
        //SmartDashboard.putNumber("AutoBrake/ffSpeed", ffSpeed);
        //SmartDashboard.putNumber("AutoBrake/targetPos", targetPos);
        //SmartDashboard.putNumber("AutoBrake/currPos", currPos);

        m_driveTrain.arcadeDrive(speed, rotation);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        m_driveTrain.stop();
    }
}
