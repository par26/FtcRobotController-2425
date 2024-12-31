package org.firstinspires.ftc.teamcode.common.subsystem;
//import com.qualcomm.robotcore.hardware.HardwareMap; ??? idk

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;
//vision!!!
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.common.vision.SampleDetectionProcessor;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;
import android.util.Size;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.common.vision.SampleDetectionPipelinePNP;

public class Vision {
    private VisionPortal visionPortal;
    public SampleDetectionProcessor visionPipeline;

    public Vision(Telemetry telemetry){
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


}
