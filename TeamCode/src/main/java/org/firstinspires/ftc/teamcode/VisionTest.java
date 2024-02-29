
package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.CSTeleOP;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.firstinspires.ftc.teamcode.TeamMarkerDetection;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

class WebCamBot{ //Webcam only, this is for testing purposes
    HardwareMap map = null;
    public void init(HardwareMap maps){
        map = maps;
    }
}
@Autonomous(name = "stream Test")
public class VisionTest extends LinearOpMode {


    private TeamMarkerDetection teamMarkerDetection;


    OpenCvCamera webcam;

    // Name of the Webcam to be set in the config
    private String webcamName = "Webcam 1";
    //imports a new robot into the file
    WebCamBot robot = new WebCamBot();
    //motor power constant in our code
    double pwr = 0.35;

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        //this line COULD be jank, shouldn't be
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        //this is what it used to be:
        //camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, webcamName), cameraMonitorViewId);
        teamMarkerDetection = new TeamMarkerDetection();
        webcam.setPipeline(teamMarkerDetection);
        //webcam.setMillisecondsPermissionTimeout(5000);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()

        {
            @Override
            public void onOpened()
            {
                webcam.startStreaming(1280,720, OpenCvCameraRotation.SENSOR_NATIVE);
            }

            @Override
            public void onError(int errorCode) {

            }
        });
        waitForStart();
        while(opModeIsActive()) {
            sleep(5000);
            webcam.stopStreaming();
        }


        while (!isStarted()) {
            telemetry.addData("ROTATION: ", teamMarkerDetection.getPosition());
            telemetry.update();

        }

    }


}


