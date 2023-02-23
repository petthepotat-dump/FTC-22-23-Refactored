package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.wrappers.Controller;
import org.firstinspires.ftc.teamcode.wrappers.DetectPoleDisplay;
import org.firstinspires.ftc.teamcode.wrappers.MecanumChassis;
import org.firstinspires.ftc.teamcode.wrappers.Position;
import org.firstinspires.ftc.teamcode.wrappers.Vision;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name="Auton Right")
public class AutonRight extends LinearOpMode {
    private MecanumChassis robot;
    private Position pos;
    private Controller control;
    private Vision sleeveDetection;
    private DetectPoleDisplay poleDetection;
    private WebcamName webcamName;
    private OpenCvCamera camera;
    private String route;

    // dashboard variables
    public static double POS1X = 0.0, POS1Y = 1.8;
    public static int MOVE1P;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new MecanumChassis(hardwareMap);
        pos = new Position(robot);
        control = new Controller(robot, pos);
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcamName = hardwareMap.get(WebcamName.class, "Camera");
        camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);
        sleeveDetection = new Vision();
        poleDetection = new DetectPoleDisplay();
        camera.setPipeline(sleeveDetection);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(320,240, OpenCvCameraRotation.SIDEWAYS_RIGHT);
            }
            @Override
            public void onError(int errorCode) {}
        });
        while (!isStarted()) {
            telemetry.addData("route: ", sleeveDetection.route);
            telemetry.update();
        }
        route = sleeveDetection.route;
        waitForStart();
        camera.setPipeline(poleDetection);
        closeIntake();
        /*
        setArmPositionTiming == wait a bit then turn on arm
         */
        // ----------------- Autonomous -----------------
        // move to position 1 (the center of our cycle);
        // do some resetting for position
        setArmPosition(520, 0.2);
        openIntake();

//        goTo(0,1.5,0,1.2,50,0.04,2,true);
        goTo(POS1X + 0.05,POS1Y + 0.02,-45,1.8,50,0.04,2,true);
        goTo(-0.18,1.55,-45,1.2,50,0.04,2,true);
//        goToPole();
        setArmPositionWait(350,0.2);
        openIntake();
        setArmPositionWait(520,0.2);
        goTo(0.05,1.4,20,0.6,180,0.08,15,false);
        setArmPositionTiming(95,0.2,0);
        goTo(0.58,1.39,90,0.6,180,0.04,2,true);
        closeIntake();
        sleep(300);
        setArmPositionWait(125, 0.2);
        setArmPositionTiming(520, 0.2, 0);
        goTo(-0.09,1.51,-45,1.2,180,0.04,2,true);
//        goToPole();
        setArmPositionWait(350, 0.2);
        openIntake();
        setArmPositionWait(520,0.2);
        goTo(0.05,1.4,20,0.6,180,0.08,15,false);
        setArmPositionTiming(85,0.2,0);
        goTo(0.61,1.39,90,0.6,240, 0.04,2,true);
        closeIntake();
        sleep(300);
        setArmPositionWait(185, 0.2);
        switch (route) {
            case "LEFT":
                setArmPositionTiming(520, 0.2, 0);
                goTo(-0.6,1.2,0,1.2,200,0.04,15,true);
                setArmPositionTiming(0, 0.2, 0);
                goTo(-0.6,0.9,0,1.2,150,0.04,2,true);
                break;
            case "CENTER":
                goTo(0,1.35,0,1.2,200,0.04,15,true);
                setArmPositionTiming(0, 0.2, 0);
                goTo(0,0.9,0,1.2,150,0.04,2,true);
                break;
            case "RIGHT":
                goTo(0.6,0.95,0,0.4, 200,0.04,2,true);
                setArmPositionTiming(0, 0.2, 0);
                goTo(0.65,0.95,0,0.4, 200,0.04,2,true);

                break;
            default:
                telemetry.addData("OH SHIT!","WE FUCKED UP!");
                telemetry.update();
                setArmPositionTiming(520, 0.2, 0);
                goTo(-0.6,1.3,0,1.2,200,0.04,15,true);
                setArmPositionTiming(0, 0.2, 0);
                goTo(-0.6,0.9,0,1.2,150,0.04,2,true);
                break;
        }
    }

    private void closeIntake() {
        robot.intake.setPosition(0.75);
    }
    private void openIntake() {
        robot.intake.setPosition(0.55);
    }
    private void goTo(double x, double y, double angle, double speed, double angleSpeed, double distanceDeadzone, double angleDeadzone, boolean velocityControl) {
        control.goTo(x, y, angle, speed, angleSpeed, distanceDeadzone, angleDeadzone, velocityControl);
        while (!control.finished) {
            if (isStopRequested()) control.stop();
            telemetry.addData("Angle: ", pos.angle);
            telemetry.addData("X: ", pos.x);
            telemetry.addData("Y: ", pos.y);
            telemetry.update();
            sleep(10);
        }
    }
    private void setArmPositionWait(int pos, double speed) {
        // move arm -- then wait untnil finished
        setArmPosition(pos, speed);
        while (!isStopRequested() && robot.leftArm.isBusy()) sleep(10);
    }
    private void setArmPositionTiming(int pos, double speed, int delay) {
        // wait -- them move
        sleep(delay);
        setArmPosition(pos, speed);
    }
    private void goToPole() {
        while (!isStopRequested() && (!(Math.abs(poleDetection.widthError) < 4 && Math.abs(poleDetection.error) < 5))) {
            robot.fl.setPower(-poleDetection.error * 0.002+poleDetection.widthError * 0.01);
            robot.fr.setPower(poleDetection.error * 0.002+poleDetection.widthError * 0.01);
            robot.bl.setPower(-poleDetection.error * 0.002+poleDetection.widthError * 0.01);
            robot.br.setPower(poleDetection.error * 0.002+poleDetection.widthError * 0.01);
            telemetry.addData("error: ", poleDetection.error);
            telemetry.addData("widthError: ", poleDetection.widthError);
            telemetry.update();
            sleep(10);
        }
    }
    private void setArmPosition(int pos, double speed){
        robot.leftArm.setTargetPosition(pos);
        robot.rightArm.setTargetPosition(pos);
        robot.leftArm.setPower(speed);
        robot.rightArm.setPower(speed);
        robot.leftArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
}
