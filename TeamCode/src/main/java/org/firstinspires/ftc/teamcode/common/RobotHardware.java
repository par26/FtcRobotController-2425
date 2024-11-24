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

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.common.subsystem.HorizontalSlides;
import org.firstinspires.ftc.teamcode.common.subsystem.Intake;
import org.firstinspires.ftc.teamcode.common.subsystem.Lift;
import org.firstinspires.ftc.teamcode.common.subsystem.Outake;
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


    //declare subsystems of the subsystem class
    Lift lift;
    Intake intake;
    Outake outake;
    HorizontalSlides horizontalSlides;


    Telemetry telemetry;


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


    private RobotHardware(final HardwareMap hardwareMap){
        subsystems = new ArrayList<>();
        lift = new Lift(hardwareMap,telemetry );
        intake = new Intake(hardwareMap);
        outake = new Outake(hardwareMap);
        horizontalSlides = new HorizontalSlides(hardwareMap, telemetry);
    }


    // initializes subsystems
    public void init() {


        lift.init();
        //intake.init();
        outake.init();
        horizontalSlides.init();


    }


    public void start() {

    }



    public void update() {

    }


    public void addSubsystem(Subsystem... subsystems) {
        this.subsystems.addAll(Arrays.asList(subsystems));
    }




}
