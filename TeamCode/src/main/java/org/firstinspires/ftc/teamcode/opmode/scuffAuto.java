//package org.firstinspires.ftc.teamcode.opmode;
//
//import static org.firstinspires.ftc.teamcode.common.pedroPathing.tuning.FollowerConstants.leftRearMotorName;
//import static org.firstinspires.ftc.teamcode.common.pedroPathing.tuning.FollowerConstants.rightFrontMotorName;
//import static org.firstinspires.ftc.teamcode.common.pedroPathing.tuning.FollowerConstants.rightRearMotorName;
//
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
//
//import org.firstinspires.ftc.teamcode.common.pedroPathing.util.Timer;
//
//import java.util.Arrays;
//import java.util.List;
//
//public class scuffAuto extends LinearOpMode {
//
//
//    private DcMotorEx leftFront;
//    private DcMotorEx leftRear;
//    private DcMotorEx rightFront;
//    private DcMotorEx rightRear;
//    private List<DcMotorEx> motors;
//
//
//
//
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
//        leftRear = hardwareMap.get(DcMotorEx.class, leftRearMotorName);
//        rightRear = hardwareMap.get(DcMotorEx.class, rightRearMotorName);
//        rightFront = hardwareMap.get(DcMotorEx.class, rightFrontMotorName);
//
//        //leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
//        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
//        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);
//
//        motors = Arrays.asList(leftFront, leftRear, rightFront, rightRear);
//
//        for (DcMotorEx motor : motors) {
//            MotorConfigurationType motorConfigurationType = motor.getMotorType().clone();
//            motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
//            motor.setMotorType(motorConfigurationType);
//        }
//
//        for (DcMotorEx motor : motors) {
//            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        }
//
//
//        while (opModeIsActive()) {
//            Timer timer = new Timer();
//            while(timer.getElapsedTimeSeconds() < 3) {
//                leftFront.setPower(-0.6);
//                rightFront.setPower(0.6);
//                leftRear.setPower(0.6);
//                rightRear.setPower(-0.6);
//            }
//        }
//    }
//}
