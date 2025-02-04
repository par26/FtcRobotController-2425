package org.firstinspires.ftc.teamcode.common.subsystem;
//import com.qualcomm.robotcore.hardware.HardwareMap; ??? idk

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;
//vision!!!
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.common.action.Action;
import org.firstinspires.ftc.teamcode.common.action.RunAction;
import org.firstinspires.ftc.teamcode.common.vision.SampleDetectionProcessor;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;
import android.util.Size;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.common.vision.SampleDetectionPipelinePNP;
import org.opencv.core.Mat;

import java.util.ArrayList;

public class Vision {
    private VisionPortal visionPortal;
    public SampleDetectionProcessor visionPipeline;

    RunAction getStone;

    public Vision(){
        this.visionPipeline = new SampleDetectionProcessor();
    }

    public void startCamera() {

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam"))
                .setCameraResolution(new Size(640, 480))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .addProcessor(visionPipeline)
                .enableLiveView(false)
                .build();

        visionPortal.setProcessorEnabled(visionPipeline, true);

    }


    public Action processSingleFrame() {
        return new Action() {
            @Override
            public boolean run(TelemetryPacket telemetryPacket) {
                if (visionPortal == null || visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
                    startCamera();
                    return false;
                }

                // Wait for the processor to process one frame
                if(visionPipeline.isFrameProcessed()) {
                    visionPortal.stopStreaming();
                    return true;
                    // Wait until the frame is processed
                }

                return false;
            }
        };
    }

    public VisionPortal.CameraState getCameraState() {
        if (visionPortal != null) return visionPortal.getCameraState();
        return null;
    }

    public void closeCamera() {
        if (visionPortal != null) visionPortal.close();
    }

    public void setProcessorEnabled(VisionProcessor processor, boolean enabled) {
        this.visionPortal.setProcessorEnabled(processor, enabled);
    }

    public ArrayList<SampleDetectionProcessor.AnalyzedStone> getAnalyzedStone() {
        return visionPipeline.getDetectedStones();
    }


}
