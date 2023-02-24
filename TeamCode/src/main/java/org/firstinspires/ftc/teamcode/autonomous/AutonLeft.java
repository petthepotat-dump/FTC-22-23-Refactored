package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.wrappers.Controller;
import org.firstinspires.ftc.teamcode.wrappers.DetectPoleDisplay;
import org.firstinspires.ftc.teamcode.wrappers.MecanumChassis;
import org.firstinspires.ftc.teamcode.wrappers.Position;
import org.firstinspires.ftc.teamcode.wrappers.Utils;
import org.firstinspires.ftc.teamcode.wrappers.Vision;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Config
@Autonomous(name="Auton Left")
public class AutonLeft extends BaseAuto {

    // dashboard variables
    public volatile static double POS1X = 0.0, POS1Y = 1.15;
    public volatile static int MOVE1P = 400, ARM1P = 600;
    public volatile static long PARKINGTIME = 4 * 10000;

    public volatile static double POS2X = -0.66, POS2Y = POS1Y,
                PIVOT1X = -0.5, PIVOT1Y = POS2Y;

    public volatile static int STAGES = 10;


    @Override
    public void runOpMode() throws InterruptedException {
        init(hardwareMap);

        route = sleeveDetection.route;
        waitForStart();

        Utils.TimerThread timer = new Utils.TimerThread(PARKINGTIME);

        closeIntake();
        camera.setPipeline(poleDetection);

        // ----------------- Autonomous ----------------- //
        // move to center position
        setArmPositionTiming(150, 0.2, 900); // arm1p

        if (STAGES < 1) return;
        setArmPositionTiming(520, 0.05, MOVE1P);
        goTo(POS1X,  POS1Y-0.2, 30, 2.0, 200, 0.04, 2, true);


        goToPole(false);

        setArmPositionWait(400, 0.2);
        openIntake();
        telemetry.addData("Stage", "CYCLE!");

        if (STAGES < 2) return;
        // next


        // begin cycle when arrive at side pylon stack
//        Utils.PylonStackTracker ptracker = new Utils.PylonStackTracker();
        // intiial == at left side (arm down h = 20)
        int heightOfStack = 20*5;
        while(heightOfStack > 0) {
            // TODO I THINK WE NEED THIS
            goTo(PIVOT1X,PIVOT1Y, 0, 0.2, 100, 0.04, 2, true);
            // then move towards left pylon stack
            setArmPosition(200,0.2);
            goTo(POS2X+0.35, POS2Y, -90, 1.7, 100/*this was 300 and i think that breaks it*/, 0.04, 2, true);
            goToCone();
            setArmPositionTiming(heightOfStack, 0.2, 300);
            goTo(pos.x-0.21, pos.y, -90, 0.4, 1, 0.03, 5, true);

            // ------------- //
            // move the robot cycle-wise as many times as possible
//            setArmPosition(ptracker.getPylonStackHeight(), 0.4);
            closeIntake();
            setArmPositionWait(100, 0.3);
            // move back a bit + spin around -- then move diagonal towards junction
            setArmPositionTiming(520, 0.3, 400);
            goTo(PIVOT1X, PIVOT1Y, 50, 1.4, 300, 0.04, 2, true);

            if (heightOfStack < 3*20) {
                goToPole(true);
            } else {
                goToPole(false);
            }
            // yeet cone down
            setArmPositionWait(400, 0.4);
            openIntake();
            heightOfStack -= 20;
         }

        if(STAGES < 3) return;
        // next

        // endgame movement
        telemetry.addData("route: ", route);
        telemetry.update();

        // removed endgame movement
        route = "NONE";

        switch (route) {
            case "LEFT":
                goTo(-0.55,1.17,0,1.2, 200,0.04,2,true);
                setArmPositionTiming(0, 0.2, 0);
                goTo(-0.55,0.85,0,1.2, 200,0.04,2,true);
                break;
            case "CENTER":
                goTo(0,1.35,0,1.8,200,0.04,15,true);
                setArmPositionTiming(0, 0.2, 0);
                goTo(0,0.9,0,1.2,150,0.04,2,true);
                break;
            case "RIGHT":
                setArmPositionTiming(520, 0.2, 0);
                goTo(0.6,1.4,0,1.8,200,0.04,15,true);
                setArmPositionTiming(0, 0.2, 0);
                goTo(0.6,0.9,0,1.2,150,0.04,2,true);
                break;
            default:
                telemetry.addData("OH SHIT!","WE FUCKED UP!");
                telemetry.update();
                goTo(-0.55,1.17,0,1.2, 200,0.04,2,true);
                setArmPositionTiming(0, 0.2, 0);
                goTo(-0.55,0.85,0,1.2, 200,0.04,2,true);
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
    private void goToRel(double x, double y, double angle, double speed, double angleSpeed, double distanceDeadzone, double angleDeadzone, boolean velocityControl) {
        goTo(pos.x + x, pos.y + y, pos.angle + angle, speed, angleSpeed, distanceDeadzone, angleDeadzone, velocityControl);
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
//    private void goToPole() {
//        while (!isStopRequested() && (!(Math.abs(poleDetection.widthError) < 4 && Math.abs(poleDetection.error) < 5))) {
//            robot.fl.setPower(-poleDetection.error * 0.002+poleDetection.widthError * 0.01);
//            robot.fr.setPower(poleDetection.error * 0.002+poleDetection.widthError * 0.01);
//            robot.bl.setPower(-poleDetection.error * 0.002+poleDetection.widthError * 0.01);
//            robot.br.setPower(poleDetection.error * 0.002+poleDetection.widthError * 0.01);
//            telemetry.addData("error: ", poleDetection.error);
//            telemetry.addData("widthError: ", poleDetection.widthError);
//            telemetry.update();
//            sleep(10);
//        }
//    }
    private void setArmPosition(int pos, double speed){
        robot.leftArm.setTargetPosition(pos);
        robot.rightArm.setTargetPosition(pos);
        robot.leftArm.setPower(speed);
        robot.rightArm.setPower(speed);
        robot.leftArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }


    private void move(double fr, double fl, double br, double bl) {
        robot.fr.setPower(fr);
        robot.fl.setPower(fl);
        robot.br.setPower(br);
        robot.bl.setPower(bl);
    }
    private void goToCone() {
        camera.setPipeline(detection);
        while (detection.x<118 || detection.x>128 || detection.width<150) {
            double error = 123-detection.x, distanceError = detection.width-150;
            double power = error/350, distancePower = distanceError/350;
            move(-power+distancePower, power+distancePower, power+distancePower, -power+distancePower);
            telemetry.addData("error", error);
            telemetry.addData("distance_error", distanceError);
            telemetry.update();
            sleep(100);
        }
    }
    private void goToPole(boolean cone) {
        camera.setPipeline(poleDetection);
        poleDetection.poleHasCone(cone);
        if (cone) {
            while (poleDetection.x<118 || poleDetection.x>128 || poleDetection.width<150) {
                double error = 123-poleDetection.x, distanceError = poleDetection.width-150;
                double power = error/350, distancePower = distanceError/350;
                move(-power+distancePower, power+distancePower, power+distancePower, -power+distancePower);
                telemetry.addData("error", error);
                telemetry.addData("distance_error", distanceError);
                telemetry.update();
                sleep(100);
            }
        } else {
            while (poleDetection.x<97 || poleDetection.x>107 || poleDetection.width<65) {
                double error = 102-poleDetection.x, distanceError = poleDetection.width-65;
                double power = error/350, distancePower = distanceError/350;
                move(-power+distancePower, power+distancePower, power+distancePower, -power+distancePower);
                telemetry.addData("error", error);
                telemetry.addData("distance_error", distanceError);
                telemetry.update();
                sleep(100);
            }
        }
    }
}
