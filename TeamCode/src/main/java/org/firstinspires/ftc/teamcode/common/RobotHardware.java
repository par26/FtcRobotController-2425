package org.firstinspires.ftc.teamcode.common;


import com.arcrobotics.ftclib.command.Subsystem;

import com.arcrobotics.ftclib.hardware.ServoEx;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.common.subsystem.Lift;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

//Serve as initlization of the robot hardware
public class RobotHardware {

    //driveTrain
    public DcMotorEx dtLeftFrontMotor;
    public DcMotorEx dtRightFrontMotor;
    public DcMotorEx dtLeftBackMotor;
    public DcMotorEx dtRightBackMotor;


    //intake
    public CRServo intakeWheel;
    public ServoEx intakePivotLeft;
    public ServoEx intakePivotRight;



    //linear slides
    public DcMotorEx liftMotor;
    public DcMotorEx liftMotor2;

    //horizontal extension slides
    public DcMotorEx horizontalExtensionMotor;

    //claw
    public ServoEx outakeClawLeft;
    public ServoEx outakeClawRight;
    public ServoEx outakeWrist

    //declare subsystems of the subsystem class
    Lift lift;



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


    // initializes subsystems
    public void init(final HardwareMap hardwareMap) {
        dtLeftBackMotor = hardwareMap.get(DcMotorEx.class, "leftFrontMotor");





        lift = new Lift();






    }

    public void read() {

    }

    public void write() {

    }


    public void periodic() {

    }


    public void addSubsystem(Subsystem... subsystems) {
        this.subsystems.addAll(Arrays.asList(subsystems));
    }




}
