package org.firstinspires.ftc.teamcode.opmode;


import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class MechanumDrive {
    private DcMotorEx frontRightMotor;
    private DcMotorEx frontLeftMotor;
    private DcMotorEx backRightMotor;
    private DcMotorEx backLeftMotor;   // private IMU imu;

    HardwareMap hardwareMap;


    public MechanumDrive(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
        init(hardwareMap);
    }
    public void init(HardwareMap hardwareMap) {
        frontLeftMotor = hardwareMap.get(DcMotorEx.class, "leftFront");
        frontRightMotor = hardwareMap.get(DcMotorEx.class, "rightFront");
        backLeftMotor = hardwareMap.get(DcMotorEx.class, "leftRear");
        backRightMotor = hardwareMap.get(DcMotorEx.class, "rightRear");
        ///imu = hardwareMap.get(IMU.class, "imu");

        //RevHubOrientationOnRobot revHubOrientationOnRobot = new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.RIGHT, RevHubOrientationOnRobot.UsbFacingDirection.UP);

        //imu.initialize(new IMU.Parameters(revHubOrientationOnRobot));


        //frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        //frontRightMotor.setDirection(DcMotor.Direction.REVERSE);
        //backRightMotor.setDirection(DcMotor.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);

        frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }

    private void setPowers(double frontRightPower, double frontLeftPower, double backLeftPower, double backRightPower) {
        frontLeftMotor.setPower(frontLeftPower);
        frontRightMotor.setPower(frontRightPower);
        backLeftMotor.setPower(backLeftPower);
        backRightMotor.setPower(backRightPower * .75);
    }

    public double squareInput(double input) {
        double result = input * input;

        if(input < 0) {
            result *= -1;
        }

        return result;
    }

    public void drive(double drive, double strafe, double turn) {
        double fLeftPow = Range.clip(drive + turn + strafe, -.85, .85);
        double bLeftPow = Range.clip(drive + turn - strafe, -.85, .85);
        double fRightPow = Range.clip(drive - turn - strafe, -.85, .85);
        double bRightPow = Range.clip(drive - turn + strafe, -.85, .85);


        setPowers(fLeftPow, fRightPow, bLeftPow, bRightPow);
    }






























}
