package org.firstinspires.ftc.teamcode.common.utils;

import com.acmerobotics.dashboard.config.Config;

@Config
public class RobotConstants {
    public static double INTAKE_ARM_LOWER = 0.45;
    public static double INTAKE_ARM_RETRACT = 0.05;
    public static double WRIST_TWIST_POSITION = 1;
    public static double OUTAKE_ARM_LOWER = 0.0;
    public static double OUTAKE_ARM_BUCKET = 1;
    public static double OUTAKE_ARM_SPECIMEN = 1;
    public static double CLAW_CLOSE = 0.5;
    public static double CLAW_OPEN = 0;



    public static double intakeSpinInPwr = 0.6;
    public static double intakeSpinOutPwr = -1;
    public static double intakeSpinStopPwr = 0;

    public static int liftToZero = 30;
    public static int liftToHighChamber = 200;
    public static int liftToLowChamber = 200;
    public static int liftToLowBucket = 200;
    public static int liftToHighBucket = 1750;
    public static int liftToTransfer = 200;

}
