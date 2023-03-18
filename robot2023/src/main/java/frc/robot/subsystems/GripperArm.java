// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class GripperArm extends SubsystemBase {
    private final Constants.HardwareConstants hardwareConstants = Constants.currentHardwareConstants();

    private final CANSparkMax m_foreArmLeader = new CANSparkMax(hardwareConstants.getForeArmLeaderId(), MotorType.kBrushed);
    private final CANSparkMax m_foreArmFollower = new CANSparkMax(hardwareConstants.getForeArmFollowerId(), MotorType.kBrushed);

    private final Solenoid m_lockElbow = new Solenoid(hardwareConstants.getPneumaticsModuleType(), hardwareConstants.getLockElbowSolenoidChannel());
    private final Solenoid m_verticalArmStage1 = new Solenoid(hardwareConstants.getPneumaticsModuleType(), hardwareConstants.getVerticalArmStage1SolenoidChannel());
    private final Solenoid m_verticalArmStage2 = new Solenoid(hardwareConstants.getPneumaticsModuleType(), hardwareConstants.getVerticalArmStage2SolenoidChannel());
    private final Solenoid m_kickstand = new Solenoid(hardwareConstants.getPneumaticsModuleType(), hardwareConstants.getKickstandSolenoidChannel());
    private final Solenoid m_gripper = new Solenoid(hardwareConstants.getPneumaticsModuleType(), hardwareConstants.getGripperSolenoidChannel());

    private final Encoder m_encoder = new Encoder(hardwareConstants.getArmEncoderChannel1(), hardwareConstants.getArmEncoderChannel2());

    private final DigitalInput m_limitSwitch = new DigitalInput(hardwareConstants.getArmLimitSwitchDigitalInput());

    private final Double FOREARM_MAX_SPEED = 0.5;

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
        resetEncoder();
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

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        SmartDashboard.putNumber("Arm/Encoder", m_encoder.getDistance());
        SmartDashboard.putString("Arm/VerticalPos", currentVerticalArmPosition.name());
        SmartDashboard.putBoolean("Arm/LimitSwitch", isForearmAtHome());
        SmartDashboard.putNumber("Arm/ForearmSpeed", getForearmSpeed());
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }

    public void resetEncoder() {
        m_encoder.reset();
    }

    public double getEncoderDistance() {
        return m_encoder.getDistance();
    }

    public VerticalArmPosition getCurrentVerticalArmPosition() {
        return currentVerticalArmPosition;
    }

    public void moveForearm(double speed) {
        if (speed > 0.0 && m_encoder.getDistance() >= currentVerticalArmPosition.maxAngleEncoderValue) {
            m_foreArmLeader.set(0.0);
        } else {
            m_foreArmLeader.set(speed);
        }
    }

    public boolean isAutoEnabled() {
        return getEncoderDistance() > currentVerticalArmPosition.autoAngleEncoderValue;
    }

    public void stopForearm() {
        m_foreArmLeader.set(0.0);
    }

    public double getForearmSpeed() {
        return m_foreArmLeader.get();
    }

    public boolean isForearmGoingUp() {
        return this.getForearmSpeed() > -0.01;
    }

    public void lockElbow() {
        m_lockElbow.set(false);
    }

    public void unlockElbow() {
        m_lockElbow.set(true);
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
        moveVerticalArm(currentVerticalArmPosition.moveForward());
    }

    public void moveVerticalArmBackward() {
        moveVerticalArm(currentVerticalArmPosition.moveBackward());
    }

    public void moveVerticalArm(VerticalArmPosition newPosition) {
        if (getEncoderDistance() < newPosition.maxAngleEncoderValue) {
            currentVerticalArmPosition = newPosition;
            positionVerticalArm();
        }
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

    public void extendKickstand() {
        m_kickstand.set(true);
    }

    public void retractKickstand() {
        m_kickstand.set(false);
    }

    public boolean isKickstandExtended() {
        return m_kickstand.get();
    }

    public void closeGripper() {
        m_gripper.set(false);
    }

    public void openGripper() {
        m_gripper.set(true);
    }

    public boolean isForearmAtHome() {
        return m_limitSwitch.get();
    }

    public enum VerticalArmPosition {
        REAR(0.5, 5.0, 22.5) {
            @Override
            public VerticalArmPosition moveForward() {
                return CENTRE;
            }

            @Override
            public VerticalArmPosition moveBackward() {
                return REAR;
            }
        },
        CENTRE(0.5, 5.0, 26) {
            @Override
            public VerticalArmPosition moveForward() {
                return FORWARD;
            }

            @Override
            public VerticalArmPosition moveBackward() {
                return REAR;
            }
        },
        FORWARD(3.0, 15.0, 31.5) {
            @Override
            public VerticalArmPosition moveForward() {
                return FORWARD;
            }

            @Override
            public VerticalArmPosition moveBackward() {
                return CENTRE;
            }
        };

        public final double maxAngleEncoderValue;
        public final double autoAngleEncoderValue;
        public final double minAngleEncoderValue;

        VerticalArmPosition(double minAngleEncoderValue, double autoAngleEncoderValue, double maxAngleEncoderValue) {
            this.minAngleEncoderValue = minAngleEncoderValue;
            this.autoAngleEncoderValue = autoAngleEncoderValue;
            this.maxAngleEncoderValue = maxAngleEncoderValue;
        }

        public abstract VerticalArmPosition moveForward();

        public abstract VerticalArmPosition moveBackward();
    }
}
