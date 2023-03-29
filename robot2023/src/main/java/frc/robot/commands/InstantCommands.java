package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.drivetrain.DriveArcade;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.GripperArm;

public class InstantCommands {

    private final GripperArm m_gripperArm;
    private final DriveTrain m_driveTrain;

    public InstantCommands(GripperArm gripperArm, DriveTrain driveTrain) {
        m_gripperArm = gripperArm;
        m_driveTrain = driveTrain;
    }

    public InstantCommand openGripper() {
        return new InstantCommand(m_gripperArm::openGripper);
    }

    public InstantCommand closeGripper() {
        return new InstantCommand(m_gripperArm::closeGripper);
    }

    public InstantCommand extendKickstand() {
        return new InstantCommand(m_gripperArm::extendKickstand);
    }

    public InstantCommand retractKickstand() {
        return new InstantCommand(m_gripperArm::retractKickstand);
    }

    public InstantCommand shiftLow() {
        return new InstantCommand(m_driveTrain::shiftLow);
    }

    public InstantCommand shiftHigh() {
        return new InstantCommand(m_driveTrain::shiftHigh);
    }
}
