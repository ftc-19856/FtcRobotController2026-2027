package Archive;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.VoltageSensor;


@Disabled
@Autonomous(name = "Archive.HandBuiltRed")
public class HandBuiltRed extends OpMode {


    double p1 = 1;
    double i1 = 0;
    double d1 = 0;

    private DcMotorEx frontLeftMotor;
    private DcMotorEx backLeftMotor;
    private DcMotorEx frontRightMotor;
    private DcMotorEx backRightMotor;

    private DcMotorEx shooterMotorOne;
    private DcMotorEx shooterMotorTwo;
    private DcMotorEx beltMotor;

    private long moveBackTimer = 0;
    private long shootTimer = 0;
    private long moveRightTimer = 0;

    private int pathState;

    public void autonomousPathUpdate(){
        switch (pathState){
            case 0:
                if(moveBackTimer == 0) {
                    moveBackTimer = System.currentTimeMillis();
                    frontLeftMotor.setPower(-.3);
                    frontRightMotor.setPower(-.3);
                    backLeftMotor.setPower(-.3);
                    backRightMotor.setPower(-.3);
                }
                else if (System.currentTimeMillis() - moveBackTimer >= 600){
                    setPathState(1);
                    frontLeftMotor.setPower(0);
                    frontRightMotor.setPower(0);
                    backLeftMotor.setPower(0);
                    backRightMotor.setPower(0);
                }
                break;

            case 1: // Wait for path to finish

                if(shootTimer == 0) {
                    shootTimer = System.currentTimeMillis();
                    shooterMotorOne.setVelocity(740);
                    shooterMotorTwo.setVelocity(740);
                }
                else if (System.currentTimeMillis() - shootTimer >= 24000){
                    setPathState(2);
                    shooterMotorOne.setVelocity(0);
                    shooterMotorTwo.setVelocity(0);
                    beltMotor.setPower(0);
                }
                else {
                    if (shooterMotorOne.getVelocity() >= 730 && shooterMotorOne.getVelocity() <= 790 && shooterMotorTwo.getVelocity() >= 730 && shooterMotorTwo.getVelocity() <= 790){
                        beltMotor.setPower(.35);
                    }
                    else beltMotor.setPower(0);
                }


            break;

            case 2: // Wait 20 seconds
                if(moveRightTimer == 0) {
                    moveRightTimer = System.currentTimeMillis();
                    frontLeftMotor.setPower(-.7);
                    frontRightMotor.setPower(.7);
                    backLeftMotor.setPower(.7);
                    backRightMotor.setPower(-.7);
                }
                else if (System.currentTimeMillis() - moveRightTimer >= 600){
                    setPathState(3);
                    frontLeftMotor.setPower(0);
                    frontRightMotor.setPower(0);
                    backLeftMotor.setPower(0);
                    backRightMotor.setPower(0);
                }
                break;

        }
    }
    public void setPathState(int pState){
        pathState = pState;
    }

    @Override
    public void init() {

        frontLeftMotor = hardwareMap.get(DcMotorEx.class, "frontLeftMotor");
        backLeftMotor = hardwareMap.get(DcMotorEx.class, "backLeftMotor");
        frontRightMotor = hardwareMap.get(DcMotorEx.class, "frontRightMotor");
        backRightMotor = hardwareMap.get(DcMotorEx.class, "backRightMotor");

        beltMotor = hardwareMap.get(DcMotorEx.class, "beltMotor");
        shooterMotorOne = hardwareMap.get(DcMotorEx.class, "shooterMotorOne");
        shooterMotorTwo = hardwareMap.get(DcMotorEx.class, "shooterMotorTwo");


        backRightMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        frontLeftMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        shooterMotorOne.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        shooterMotorTwo.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        backRightMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        frontRightMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        backLeftMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        frontLeftMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        shooterMotorOne.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        shooterMotorTwo.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        shooterMotorOne.setDirection(DcMotorEx.Direction.REVERSE);
        shooterMotorTwo.setDirection(DcMotorSimple.Direction.FORWARD);

        shooterMotorOne.setVelocityPIDFCoefficients(p1, i1, d1, 13.5);
        shooterMotorTwo.setVelocityPIDFCoefficients(p1, i1, d1, 13.5);


    }

    @Override
    public void loop() {
        if (pathState == 1) { // shooter running
            double voltage = getBatteryVoltage();

            double compensatedF = 13.5 * (12.0 / voltage);

            shooterMotorOne.setVelocityPIDFCoefficients(p1, i1, d1, compensatedF);
            shooterMotorTwo.setVelocityPIDFCoefficients(p1, i1, d1, compensatedF);

            telemetry.addData("Battery V", voltage);
            telemetry.addData("Shooter F", compensatedF);
        }
        autonomousPathUpdate();
        telemetry.addData("Path state", pathState);
        telemetry.addData("Shooter motor 1 spd", shooterMotorOne.getVelocity());
        telemetry.addData("Shooter motor 2 spd", shooterMotorTwo.getVelocity());
        telemetry.addData("Shooter time", (System.currentTimeMillis()-shootTimer)/1000);
        telemetry.update();
    }

    private double getBatteryVoltage(){
        for (VoltageSensor sensor : hardwareMap.voltageSensor) {
            double v = sensor.getVoltage();
            if (v > 11.0) return v; // ignore 5V logic rail
        }
        return 12.0; // safe fallback
    }


}
