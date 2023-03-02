// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static edu.wpi.first.wpilibj.PneumaticsModuleType.CTREPCM;

public class GripperArm extends SubsystemBase {
    private final CANSparkMax m_foreArmLeader = new CANSparkMax(42, MotorType.kBrushed);
    private final CANSparkMax m_foreArmFollower = new CANSparkMax(43, MotorType.kBrushed);

    private final Solenoid m_lockElbow = new Solenoid(CTREPCM, 0);
    private final Solenoid m_verticalArmStage1 = new Solenoid(CTREPCM, 1);
    private final Solenoid m_verticalArmStage2 = new Solenoid(CTREPCM, 2);
    private final Solenoid m_gripper = new Solenoid(CTREPCM, 6);

    private final Encoder m_encoder = new Encoder(0, 1);

    private VerticalArmPosition currentVerticalArmPosition = VerticalArmPosition.REAR;

    /**
     * Creates a new GripperArm.
     */
    public GripperArm() {
        m_foreArmLeader.restoreFactoryDefaults();
        m_foreArmFollower.restoreFactoryDefaults();

        m_foreArmLeader.setIdleMode(IdleMode.kBrake);
        m_foreArmFollower.setIdleMode(IdleMode.kBrake);

        m_foreArmFollower.follow(m_foreArmLeader);

        m_encoder.reset();
    }

    public void initTeleop() {
        m_encoder.reset();
        positionVerticalArm();
    }

    /**
     * Example command factory method.
     *
     * @return a command
     */
    public CommandBase exampleMethodCommand() {
        // Inline construction of command goes here.
        // Subsystem::RunOnce implicitly requires `this` subsystem.
        return runOnce(
                () -> {
                    /* one-time action goes here */
                });
    }

    /**
     * An example method querying a boolean state of the subsystem (for example, a digital sensor).
     *
     * @return value of some boolean subsystem state, such as a digital sensor.
     */
    public boolean exampleCondition() {
        // Query some boolean state, such as a digital sensor.
        return false;
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        SmartDashboard.putNumber("Arm/Encoder", m_encoder.getDistance());
        SmartDashboard.putString("Arm/VerticalPos", currentVerticalArmPosition.name());
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }

    public void moveForearm(double speed) {
        if (speed > 0.0 && m_encoder.getDistance() >= currentVerticalArmPosition.maxAngleEncoderValue) {
            m_foreArmLeader.set(0.0);
        } else {
            m_foreArmLeader.set(speed);
        }
    }

    public void upForearm() {
        if (m_encoder.getDistance() >= currentVerticalArmPosition.maxAngleEncoderValue) {
            m_foreArmLeader.set(0);
        } else {
            m_foreArmLeader.set(1.0);
        }
    }

    public void downForeArm() {
        m_foreArmLeader.set(-1.0);
    }

    public void stopForearm() {
        m_foreArmLeader.set(0.0);
    }

    public void lockElbow() {
        m_lockElbow.set(true);
    }

    public void unlockElbow() {
        m_lockElbow.set(false);
    }

    private void enableStage1() {
        m_verticalArmStage1.set(true);
    }

    private void disableStage1() {
        m_verticalArmStage1.set(false);
    }

    private void enableStage2() {
        m_verticalArmStage2.set(true);
    }

    private void disableStage2() {
        m_verticalArmStage2.set(false);
    }

    public void moveVerticalArmForward() {
        currentVerticalArmPosition = currentVerticalArmPosition.moveForward(m_encoder.getDistance());
        positionVerticalArm();
    }

    public void moveVerticalArmBackward() {
        currentVerticalArmPosition = currentVerticalArmPosition.moveBackward(m_encoder.getDistance());
        positionVerticalArm();
    }

    private void positionVerticalArm() {
        switch (currentVerticalArmPosition) {
            case REAR:
                disableStage1();
                disableStage2();
                break;
            case CENTRE:
                enableStage1();
                disableStage2();
                break;
            case FORWARD:
                enableStage1();
                enableStage2();
                break;
        }
    }

    public enum VerticalArmPosition {
        REAR(22.5) {
            @Override
            public VerticalArmPosition moveForward(double angleEncoderValue) {
                return CENTRE;
            }

            @Override
            public VerticalArmPosition moveBackward(double angleEncoderValue) {
                return REAR;
            }
        },
        CENTRE(26) {
            @Override
            public VerticalArmPosition moveForward(double angleEncoderValue) {
                return FORWARD;
            }

            @Override
            public VerticalArmPosition moveBackward(double angleEncoderValue) {
                // empèche de reculer le bras dans une position qui bloquerait
                if (angleEncoderValue > REAR.maxAngleEncoderValue) {
                    return this;
                }
                return REAR;
            }
        },
        FORWARD(31.5) {
            @Override
            public VerticalArmPosition moveForward(double angleEncoderValue) {
                return FORWARD;
            }

            @Override
            public VerticalArmPosition moveBackward(double angleEncoderValue) {
                // empèche de reculer le bras dans une position qui bloquerait
                if (angleEncoderValue > CENTRE.maxAngleEncoderValue) {
                    return this;
                }
                return CENTRE;
            }
        };

        public final double maxAngleEncoderValue;

        VerticalArmPosition(double value) {
            maxAngleEncoderValue = value;
        }

        public abstract VerticalArmPosition moveForward(double angleEncoderValue);

        public abstract VerticalArmPosition moveBackward(double angleEncoderValue);
    }
}
