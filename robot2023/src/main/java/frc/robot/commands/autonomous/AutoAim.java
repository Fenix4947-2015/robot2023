/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.autonomous;

import java.util.Objects;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.SmartDashboardSettings;
import frc.robot.limelight.Limelight;
import frc.robot.subsystems.DriveTrain;

public class AutoAim extends CommandBase {
  public static final double K_FEED_FORWARD_ANGLE = 0.23;
  public static final double K_PID_P_ANGLE = 0.018;
  public static final double K_PID_I_ANGLE = 0.000;
  public static final double K_PID_D_ANGLE = 0.000;

  public static final double K_FEED_FORWARD_DISTANCE = 0.0;
  public static final double K_PID_P_DISTANCE = 0.35;
  public static final double K_PID_I_DISTANCE = 0.0;
  public static final double K_PID_D_DISTANCE = 0.0;

  public static final String PIDTYPE_AUTOAIM = "AUTOAIM";

  public static final int AUTOAIM_PICK_PIPELINE = 0;
  public static final int AUTOAIM_PLACE_PIPELINE = 2;

  private final DriveTrain _driveTrain;
  private final Limelight _limelight;

  private final int _pipeline;

  public double _driveCommand = 0.0;
  public double _steerCommand = 0.0;
  private PIDController _pidAngle = new PIDController(K_PID_P_ANGLE, K_PID_I_ANGLE, K_PID_D_ANGLE);
  private PIDController _pidDistance = new PIDController(K_PID_P_DISTANCE, K_PID_I_DISTANCE, K_PID_D_DISTANCE);
  private double _feedForward = K_FEED_FORWARD_ANGLE;

  private boolean _isAtSetPoint = false;

  public AutoAim(int pipeline, DriveTrain driveTrain, Limelight limelight) {
    _pipeline = pipeline;
    _driveTrain = driveTrain;
    _limelight = limelight;
    addRequirements(_driveTrain, _limelight);
  }

  // Called just before this Command runs the first time
  @Override
  public void initialize() {
    //System.out.println("AutoAim");
    _driveTrain.shiftLow();
    _isAtSetPoint = false;
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() {
    _limelight.changePipeline(_pipeline);
    //refreshPidValues();
    updateTracking();
    SmartDashboard.putNumber("AutoAim/_driveCommand", _driveCommand);
    SmartDashboard.putNumber("AutoAim/_steerCommand", _steerCommand);

    if (_limelight.isTargetValid()) {
      _driveTrain.arcadeDrive(-_driveCommand, _steerCommand);
    } else {
      _driveTrain.stop();
    }

  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  public boolean isFinished() {
    return _isAtSetPoint;
  }

  // Called once after isFinished returns true
  @Override
  public void end(boolean interrupted) {
    _driveTrain.stop();
  }

  public void setAnglePID(double p, double i, double d, double f) {
    // System.out.println(String.format("pid: %f %f %f %f", p, i, d, f));
    _pidAngle.setPID(p, i, d);
    _feedForward = f;
  }

  public void updateTracking() {
    _driveCommand = 0.0;
    _steerCommand = 0.0;

    // These numbers must be tuned for your Robot! Be careful!
    final double DESIRED_HEIGHT = 0.0; //8.6;
    final double DESIRED_ANGLE = 0.0;//2.6;
    final double MAX_DRIVE = 0.7; // Simple speed limit so we don't drive too fast

    final boolean tv = _limelight.isTargetValid();
    final double tx = _limelight.getTx();
    final double ty = _limelight.getTy();

    //System.out.println(String.format("tv: %s, tx: %f, ty: %f", tv, tx, ty));

    if (!tv) {
      return;
    }

    _pidAngle.setSetpoint(DESIRED_ANGLE);
    _pidAngle.setTolerance(1.0);
    double steer_cmd = _pidAngle.calculate(tx);

    double feedFwd = Math.signum(steer_cmd) * _feedForward;
    _steerCommand = steer_cmd + feedFwd;

    _pidDistance.setSetpoint(DESIRED_HEIGHT);
    _pidDistance.setTolerance(2.0);
    double drive_cmd = _pidDistance.calculate(ty);

    // try to drive forward until the target area reaches our desired area
    // double drive_cmd = (DESIRED_TARGET_AREA - ta) * DRIVE_K;
    //double drive_cmd = (DESIRED_HEIGHT - ty) * -DRIVE_K;

    steer_cmd = Math.max(steer_cmd, -0.5);
    drive_cmd = Math.max(drive_cmd, -0.5);

    steer_cmd = Math.min(steer_cmd, 0.5);
    drive_cmd = Math.min(drive_cmd, 0.5);

    // don't let the robot drive too fast into the goal
    if (drive_cmd > MAX_DRIVE) {
      drive_cmd = MAX_DRIVE;
    }
    _driveCommand = drive_cmd;

    _pidAngle.atSetpoint();
    _isAtSetPoint = _pidAngle.atSetpoint() && _pidDistance.atSetpoint();

  }
  

}
