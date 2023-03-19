package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.GripperArm;

public class InstantCommands {

    private final GripperArm m_gripperArm;

    public InstantCommands(GripperArm gripperArm) {
        m_gripperArm = gripperArm;
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
}
