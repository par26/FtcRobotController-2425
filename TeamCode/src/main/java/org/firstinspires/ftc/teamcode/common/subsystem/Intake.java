package org.firstinspires.ftc.teamcode.common.subsystem;

import androidx.annotation.NonNull;

import com.arcrobotics.ftclib.command.Subsystem;
import com.arcrobotics.ftclib.controller.PDController;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.common.RobotHardware;

public class Intake implements Subsystem {

    private final RobotHardware robot;

//    public static double
    //          kP = 0.0031, kI = 0, kD = 0.0001, kF = 0.00;
    // PID values
// used for making sure the active intake does not die immediately
    // also tune later

    double

    enum State {
        forward, reverse, stop
    }

    enum ARM {
        lower, retract
    }



    Intake.State state;
    PDController intakePID;

    Intake.ARM armState;

    //double PID;
    double intakeSpeed = 0.8;
    double power;
    double lastPower;
    double wristServoPosition;

    public Intake() {
        this.robot = RobotHardware.getInstance();

        state = Intake.State.stop;
        power = 0;
        lastPower = 0;
        //clean slate, probably

    }


    @Override
    public void periodic() {
        switch(state) {
            case forward:
                setPower(intakeSpeed);
            case reverse:
                setPower(-intakeSpeed);
            case stop:
                setPower(0);
        }
    }

    public void startIntake() {
        state = state.forward;

        //pid probably should have  avalue before this starts up
    }

    public void reverseIntake() {
        state = state.reverse;
        //PID should go in reverse, so yeah also psuedocode
    }
    public void stopIntake() {
        state = state.stop;

        //PID should go in reverse, so yeah also psuedocode
    }

    /*
    Gives us the option to implement PID in the future
     */
    private void setPower(double power) {
        robot.intakeWheel.setPower(power);
    }


    public void lowerIntake() {
        //there is a wrist

        robot.intakePivotLeft.setPosition(ARM_LOWER);
        robot.intakePivotRight.setPosition(ARM_LOWER);
    }
    public void retractIntake() {
        //bw
        robot.intakePivotLeft.setPosition(ARM_RETRACT);
        robot.intakePivotRight.setPosition(ARM_RETRACT);
    }
    public void twistIntake() {
        robot.wrist.setPosition(wristServoPosition);
    }
}