package org.firstinspires.ftc.teamcode.common;


import com.arcrobotics.ftclib.command.Subsystem;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.Arrays;
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

    //horizontal extension slides
    DcMotorEx horizontalExtensionMotor;

    //claw
    Servo outakeClawLeft;
    Servo outakeClawRight;


    //declare subsystems of the subsystem class




    HardwareMap hardwareMap;


    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;

    private ElapsedTime voltageTimer = new ElapsedTime();
    private double voltage = 12.0;


    private static RobotHardware instance = null;
    private boolean enabled;

    public List<LynxModule> modules;
    public LynxModule CONTROL_HUB;


    // list of all subsystems
    ArrayList<Subsystem> subsystems;

    public static RobotHardware getInstance() {
        if(instance == null) {
            instance = new RobotHardware();
        }
        return instance;
    }

    private RobotHardware(){
        subsystems = new ArrayList<>();
    }

    public void create(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
    }

    // initializes subsystems
    public void init() {
        dtLeftBackMotor = new DcMotorEx("leftFrontMotor");
    }

    public void read() {

        for(Subsystem subsystem : subsystems) {
            subsystem.read();
        }
    }

    public void write() {
        for(Subsystem subsystem : subsystems) {
            subsystem.write();
        }
    }


    public void periodic() {

    }


    public void addSubsystem(Subsystem... subsystems) {
        this.subsystems.addAll(Arrays.asList(subsystems));
    }




}
