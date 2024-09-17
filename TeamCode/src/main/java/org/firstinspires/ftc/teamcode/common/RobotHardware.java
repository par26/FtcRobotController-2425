package org.firstinspires.ftc.teamcode.common;


import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

//Serve as initlization of the robot hardware
public class RobotHardware {

    //driveTrain
    DcMotorEx dtLeftFrontMotor;
    DcMotorEx dtRightFrontMotor;
    DcMotorEx dtLeftBackMotor;
    DcMotorEx dtRightBackMotor;


    //linear slides
    DcMotorEx verticalExtensionMotor;

    //claw
    Servo outakeClawLeft;
    Servo outakeClawRight;


    //declare subsystems of the subsystem class



    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;

    private ElapsedTime voltageTimer = new ElapsedTime();
    private double voltage = 12.0;


    private static RobotHardware instance = null;
    private boolean enabled;

    public List<LynxModule> modules;
    public LynxModule CONTROL_HUB;


    public void init(HardwareMap hwMap) {

    }


    public void periodic() {

    }




}
