// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.gripperarm;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.GripperArm;

/**
 * An example command that uses an example subsystem.
 */
public class HomeForearm extends CommandBase {

    private enum State {
        INIT,
        UNLOCKED,
        LOCKED,
        LOCKING,
        HOMED
    }

    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final GripperArm m_gripperArm;
    private State state;
    private Timer timer = new Timer();

    /**
     * Creates a new ExampleCommand.
     *
     * @param subsystem The subsystem used by this command.
     */
    public HomeForearm(GripperArm gripperArm) {
        m_gripperArm = gripperArm;
        state = State.INIT;
        timer.reset();
    
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(gripperArm);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        state = State.INIT;
        m_gripperArm.moveForearm(0);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (state == State.INIT) {
            m_gripperArm.unlockElbow();
            m_gripperArm.moveForearm(0);
            state = State.UNLOCKED;
        } else if (state == State.UNLOCKED) {
            if (m_gripperArm.isElbowUp()) {
                timer.reset();
                timer.start();
            } else if (timer.get() > 0.25) {
                state = State.LOCKING;
                m_gripperArm.lockElbow();
                timer.reset();
                timer.start();
            }
        } else if (state == State.LOCKING) { 
            if (timer.get() > 0.01) {
                state = State.LOCKED;
                timer.reset();
                timer.stop();
            }
        }
        else if (state == State.LOCKED) {
            if (m_gripperArm.isForearmAtHome() && m_gripperArm.isForearmGoingUp()) {
                m_gripperArm.moveForearm(1.0);
            } else if (!m_gripperArm.isForearmAtHome()) {
                m_gripperArm.moveForearm(-0.5);
            } else {
                state = State.HOMED;
            }
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_gripperArm.resetEncoder();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return (state == State.HOMED);
    }
}
