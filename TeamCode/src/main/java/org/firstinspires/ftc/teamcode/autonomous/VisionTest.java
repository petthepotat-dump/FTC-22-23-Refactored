package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.wrappers.DetectConeDisplay;
import org.firstinspires.ftc.teamcode.wrappers.DetectPole;
import org.firstinspires.ftc.teamcode.wrappers.DetectPoleDisplay;
import org.firstinspires.ftc.teamcode.wrappers.DisplayVision;
import org.firstinspires.ftc.teamcode.wrappers.MecanumChassis;
import org.firstinspires.ftc.teamcode.wrappers.Vision;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name = "VisionTest")
public class VisionTest extends LinearOpMode {
    private OpenCvCamera camera;
    private FtcDashboard dashboard = FtcDashboard.getInstance();
//    private DetectConeDisplay detection = new DetectConeDisplay(true);
    private DetectPoleDisplay poleDetection = new DetectPoleDisplay();
    private DisplayVision detection = new DisplayVision();
    private MecanumChassis robot;
    @Override
    public void runOpMode() throws InterruptedException {
        robot = new MecanumChassis(hardwareMap);
        telemetry = dashboard.getTelemetry();
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Camera"), cameraMonitorViewId);
        poleDetection.poleHasCone(false);
        camera.setPipeline(detection);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(320, 240, OpenCvCameraRotation.SIDEWAYS_LEFT);
                dashboard.startCameraStream(camera, 30);
            }

            @Override
            public void onError(int errorCode) {
            }
        });
        while (!isStarted()) {
            telemetry.addData("hue", detection.hsvColor[0]);
            telemetry.addData("Saturation", detection.hsvColor[1]);
            telemetry.addData("Value", detection.hsvColor[2]);
//            telemetry.addData("x", poleDetection.x);
//            telemetry.addData("y", poleDetection.y);
//            telemetry.addData("height", poleDetection.height);
//            telemetry.addData("width", poleDetection.width);

            telemetry.update();
            sleep(200);
        }
        camera.closeCameraDeviceAsync(() -> {
            dashboard.stopCameraStream();
        });
    }
    private void move(double fr, double fl, double br, double bl) {
        robot.fr.setPower(fr);
        robot.fl.setPower(fl);
        robot.br.setPower(br);
        robot.bl.setPower(bl);
    }
}