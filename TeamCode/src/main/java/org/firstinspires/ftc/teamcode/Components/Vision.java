package org.firstinspires.ftc.teamcode.Components;

import android.util.Size;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.acmerobotics.dashboard.config.Config;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;

@Config
public class Vision {
    private TfodProcessor tfod;

    private VisionPortal visionPortal;
    private static final String TFOD_MODEL_ASSET_BLUE = "blueboxdetectorcenterstage.tflite";
    private static final String TFOD_MODEL_ASSET_RED = "redboxdetectorcenterstage.tflite";

    /**           IF YOU ARE GOING TO CHANGE THE MODEL CHANGE THIS FILE NAME
     *
     *         The location of the file that the model is stored in is FtcRobotController/assets
     * */
    private static final String[] LABELS = {
            "Box", /**THE NAME OF THE LABEL ASSIGNED TO OUR TEAM MARKER*/
    };
    private int leftBound = 400;/** variables for later classification */
    private int rightBound = 600;
    private List<Recognition> currentRecognitions;
    public Vision (HardwareMap hardwareMap, boolean isBlue){
        initTfod(hardwareMap, isBlue);
    }


    private void initTfod(HardwareMap hardwareMap, boolean isBlue) {
        if(isBlue){
            tfod = new TfodProcessor.Builder()
                    .setModelAssetName(TFOD_MODEL_ASSET_BLUE)
                    .setModelLabels(LABELS)
                    //.setModelAspectRatio(16.0 / 9.0)
                    .build();
        } else{
            tfod = new TfodProcessor.Builder()
                    .setModelAssetName(TFOD_MODEL_ASSET_RED)
                    .setModelLabels(LABELS)
                    //.setModelAspectRatio(16.0 / 9.0)
                    .build();
        }

        VisionPortal.Builder builder = new VisionPortal.Builder();

        builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));

        builder.setCameraResolution(new Size(1280, 720)); // model is in 720p, but can be used on various resolutions
        // on our webcam it runs when you put in 1280x720,
        // but it says that it's unsupported and auto-chooses a new resolution with the same aspect ratio

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        builder.enableLiveView(true);

        builder.setStreamFormat(VisionPortal.StreamFormat.MJPEG);

        builder.setAutoStopLiveView(false);
        builder.addProcessor(tfod);


        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();
        // if not confident its a box, dont show it
        tfod.setMinResultConfidence(0.70f);
    }
    public void open(){
        visionPortal.setProcessorEnabled(tfod, true);
    }
    public void close(){
        visionPortal.setProcessorEnabled(tfod,false);
    }
    public double x1, y1, x2, y2;
    public void findFirstBox() {
        currentRecognitions = tfod.getRecognitions();
        for (Recognition recognition : currentRecognitions) {//if you think this is jank, i dont wanna hear it
            x1 = recognition.getLeft();
            y1 = recognition.getRight();
            x2 = recognition.getTop();
            y2 = recognition.getBottom();
            break;// idk if anyone is gonna read this but i <3 breaks
        }
    }
    public int getSpike(){ /** THIS IS HOW WE ARE CLASSIFYING THE SPIKE*/
        findFirstBox();
        if (x2 <= leftBound){
            return 1;
        } else if (x1>=rightBound) {
            return 3;
        }
        return 2;
    }
}
