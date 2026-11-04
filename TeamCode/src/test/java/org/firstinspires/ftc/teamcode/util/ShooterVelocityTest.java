package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import java.lang.reflect.Proxy;
import org.junit.Before;
import org.junit.Test;
import static org.junit.Assert.*;

public class ShooterVelocityTest {
    private double velocity;
    private double power;
    private int velocityCommands;
    private int gainWrites;
    private double appliedP;
    private DcMotor.RunMode mode;
    private DcMotor.ZeroPowerBehavior zeroPower;
    private ShooterVelocity shooter;

    @Before public void setup() {
        MotorConfigurationType type = new MotorConfigurationType();
        type.setTicksPerRev(28);
        type.setMaxRPM(6000);
        DcMotorEx motor = (DcMotorEx) Proxy.newProxyInstance(DcMotorEx.class.getClassLoader(),
                new Class<?>[] {DcMotorEx.class}, (proxy, method, args) -> {
                    switch (method.getName()) {
                        case "getMotorType": return type;
                        case "getPIDFCoefficients": return new PIDFCoefficients(10, 3, 0, 12);
                        case "setMode": mode = (DcMotor.RunMode) args[0]; break;
                        case "setZeroPowerBehavior": zeroPower = (DcMotor.ZeroPowerBehavior) args[0]; break;
                        case "setPower":
                            power = (double) args[0];
                            if (power == 0) velocity = 0;
                            break;
                        case "setVelocity": velocity = (double) args[0]; velocityCommands++; break;
                        case "setVelocityPIDFCoefficients": appliedP = (double) args[0]; gainWrites++; break;
                        default: break;
                    }
                    return null;
                });
        shooter = new ShooterVelocity(motor);
        ShooterVelocity.P = 10;
        ShooterVelocity.I = 3;
        ShooterVelocity.D = 0;
        ShooterVelocity.F = 12;
        ShooterVelocity.SPEED_FRACTION = 1;
    }

    @Test public void initDoesNotCommandMotionAndUsesCoastingVelocityMode() {
        assertEquals(0, power, 0);
        assertEquals(0, velocityCommands);
        assertEquals(DcMotor.RunMode.RUN_USING_ENCODER, mode);
        assertEquals(DcMotor.ZeroPowerBehavior.FLOAT, zeroPower);
    }

    @Test public void requestsFullConfiguredSpeedInTheWorkingDirection() {
        shooter.setEnabled(true);
        assertEquals(-2800, velocity, 0.001);
        ShooterVelocity.SPEED_FRACTION = 0.75;
        shooter.update();
        assertEquals(-2100, velocity, 0.001);
    }

    @Test public void stoppingCannotRestartOnNextAutonomousUpdate() {
        shooter.setEnabled(true);
        shooter.stop();
        int previousCommands = velocityCommands;
        shooter.update();
        assertEquals(0, power, 0);
        assertEquals(0, velocity, 0);
        assertEquals(previousCommands, velocityCommands);
    }

    @Test public void appliesLiveGainsWithoutRewritingThemEachLoop() {
        shooter.setEnabled(true);
        shooter.update();
        assertEquals(1, gainWrites);
        ShooterVelocity.P = 15;
        shooter.update();
        assertEquals(2, gainWrites);
        assertEquals(15, appliedP, 0);
    }

    @Test public void invalidSettingsStopTheMotorInsteadOfSendingInvalidCommands() {
        shooter.setEnabled(true);
        ShooterVelocity.SPEED_FRACTION = Double.NaN;
        shooter.update();
        assertEquals(0, velocity, 0);
        ShooterVelocity.SPEED_FRACTION = 1;
        ShooterVelocity.P = -1;
        shooter.update();
        assertEquals(0, velocity, 0);
        assertEquals(1, gainWrites);
    }
}
