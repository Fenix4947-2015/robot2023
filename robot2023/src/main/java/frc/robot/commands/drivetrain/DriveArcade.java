package frc.robot.commands.drivetrain;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.exchange.SubSystemDataExchange;
import frc.robot.subsystems.DriveTrain;

public class DriveArcade extends CommandBase {

    private final XboxController m_controller;
    private final DriveTrain m_driveTrain;

    private final Constants.HardwareConstants hardwareConstants = Constants.currentHardwareConstants();

    private final SlewRateLimiter m_slewRateLimiter = new SlewRateLimiter(hardwareConstants.getDriveSlewRate());

    private final SubSystemDataExchange m_dataExchange;

    public DriveArcade(XboxController controller, DriveTrain driveTrain, SubSystemDataExchange dataExchange) {
        m_controller = controller;
        m_driveTrain = driveTrain;
        m_dataExchange = dataExchange;

        addRequirements(driveTrain);
    }

    @Override
    public void execute() {
        double speed = m_controller.getRightTriggerAxis() - m_controller.getLeftTriggerAxis();
        double rotation = -m_controller.getLeftX();

        speed = m_dataExchange.getSpeedLimited() ? 0.6 * speed : speed;
        rotation = m_dataExchange.getSpeedLimited() ? 0.6 * rotation : rotation;

        double rampedSpeed = m_slewRateLimiter.calculate(speed);

        m_driveTrain.arcadeDrive(rampedSpeed, rotation);
    }
}
