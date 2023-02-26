package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.wrappers.Controller;
import org.firstinspires.ftc.teamcode.wrappers.DetectConeDisplay;
import org.firstinspires.ftc.teamcode.wrappers.DetectPoleDisplay;
import org.firstinspires.ftc.teamcode.wrappers.DisplayVision;
import org.firstinspires.ftc.teamcode.wrappers.MecanumChassis;
import org.firstinspires.ftc.teamcode.wrappers.Position;
import org.firstinspires.ftc.teamcode.wrappers.Vision;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

public abstract class BaseAuto extends LinearOpMode {
    public MecanumChassis robot;
    public Position pos;
    public Controller control;
    public DisplayVision sleeveDetection;
    public DetectPoleDisplay poleDetection;
    public DetectConeDisplay coneDetection;
    public WebcamName webcamName;
    public OpenCvCamera camera;
    public FtcDashboard dashboard = FtcDashboard.getInstance();

    public void init(HardwareMap map, boolean blueTeam){
        robot = new MecanumChassis(map);
        pos = new Position(robot);
        control = new Controller(robot, pos);
        int cameraMonitorViewId = map.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcamName = hardwareMap.get(WebcamName.class, "Camera");
        camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);
        sleeveDetection = new DisplayVision();
        poleDetection = new DetectPoleDisplay();
        coneDetection = new DetectConeDisplay(blueTeam); // TODO depends on team
        camera.setPipeline(sleeveDetection);
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
            telemetry.addData("route: ", sleeveDetection.route);
            telemetry.update();
        }
    }
    public void closeIntake() {
        robot.intake.setPosition(0.78);
    }
    public void openIntake() {
        robot.intake.setPosition(0.55);
    }
    public void goTo(double x, double y, double angle, double speed, double angleSpeed, double distanceDeadzone, double angleDeadzone, boolean velocityControl) {
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
    public void goToRel(double x, double y, double angle, double speed, double angleSpeed, double distanceDeadzone, double angleDeadzone, boolean velocityControl) {
        goTo(pos.x+x, pos.y+y, pos.angle+angle, speed, angleSpeed, distanceDeadzone, angleDeadzone, velocityControl);
    }
    public void setArmPositionWait(int pos, double speed) {
        setArmPosition(pos, speed);
        while (!isStopRequested() && robot.leftArm.isBusy()) sleep(10);
    }
    public void setArmPositionTiming(int pos, double speed, int delay) {
        sleep(delay);
        setArmPosition(pos, speed);
    }
    public void setArmPosition(int pos, double speed){
        robot.leftArm.setTargetPosition(pos);
        robot.rightArm.setTargetPosition(pos);
        robot.leftArm.setPower(speed);
        robot.rightArm.setPower(speed);
        robot.leftArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void move(double fr, double fl, double br, double bl) {
        robot.fr.setPower(fr);
        robot.fl.setPower(fl);
        robot.br.setPower(br);
        robot.bl.setPower(bl);
    }
    public void goToCone(int armPos) {
        camera.setPipeline(coneDetection);
        while (coneDetection.x<113 || coneDetection.x>133 || coneDetection.width<150) {
            double error = 123-coneDetection.x, distanceError = coneDetection.width-150;
            double power = error/310, distancePower = distanceError/350;
            if (coneDetection.left < 4 || coneDetection.right > 236) move(-power+0.05, power+0.05, power+0.05, -power+0.05);
            move(-power+distancePower, power+distancePower, power+distancePower, -power+distancePower);
            telemetry.addData("error", error);
            telemetry.addData("distance_error", distanceError);
            telemetry.update();
            sleep(100);
        }
        setArmPosition(armPos, 0.4);
        double ticks = robot.fr.getCurrentPosition();
        move(-0.08, -0.08, -0.08, -0.08);
        while (ticks-robot.fr.getCurrentPosition()<150) sleep(50);
        move(0,0,0,0);

    }
    public void goToPole(boolean cone) {
        camera.setPipeline(poleDetection);
        poleDetection.poleHasCone(cone);
        if (cone) {
            while (poleDetection.x<118 || poleDetection.x>128 || poleDetection.width<150) {
                double error = 123-poleDetection.x, distanceError = poleDetection.width-150;
                double power = error/350, distancePower = distanceError/300;
                move(-power+distancePower, power+distancePower, power+distancePower, -power+distancePower);
                telemetry.addData("error", error);
                telemetry.addData("distance_error", distanceError);
                telemetry.update();
                sleep(100);
            }
            double ticks = robot.fr.getCurrentPosition();
            move(-0.08, -0.08, -0.08, -0.08);
            while (ticks-robot.fr.getCurrentPosition()<140) sleep(50);
            move(0,0,0,0);
        } else {
            while (poleDetection.x<85 || poleDetection.x>119 || poleDetection.width<60) {
                double error = 102-poleDetection.x, distanceError = poleDetection.width-60;
                double power = error/270, distancePower = distanceError/340;
                power = Math.signum(power)*Math.max(Math.min(Math.abs(power), 0.13), 0.06);
                distancePower = Math.signum(distancePower)*Math.max(Math.abs(distancePower), 0.05);
                move(-power+distancePower, power+distancePower, power+distancePower, -power+distancePower);
                telemetry.addData("error", error);
                telemetry.addData("distance_error", distanceError);
                telemetry.update();
                sleep(100);
            }

        }
    }
    @Override
    public abstract void runOpMode() throws InterruptedException;

}
