package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.wrappers.Controller;
import org.firstinspires.ftc.teamcode.wrappers.MecanumChassis;
import org.firstinspires.ftc.teamcode.wrappers.Position;

@Config
@TeleOp(name="ChassisConfig")
public class ChassisControlTest extends LinearOpMode {
    // we have (target) t values
    // and (actual) a values
    public volatile static double tx, ty, ttheta, speed = 1.0, angleSpeed = 30.0, moveDZ = 1.0, angleDZ = 1.0;
    private MecanumChassis robot;
    private Position pos;
    private Controller controller;
    @Override
    public void runOpMode() {
        robot = new MecanumChassis(hardwareMap);
        pos = new Position(robot);
        controller = new Controller(robot, pos);
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        dashboard.updateConfig();
        waitForStart();
        /*
            For determining strafe + movement ratio (movement not usually necassary but could be tuned)
            - dashboard set target + speed + angleSpeed
         */
        controller.goTo(tx, ty, ttheta, speed, angleSpeed, moveDZ, angleDZ, true);
        while (!controller.finished) {
            if (isStopRequested()) controller.stop();
            telemetry.addData("ax", pos.x);
            telemetry.addData("ay", pos.y);
            telemetry.addData("atheta", pos.angle);
            telemetry.addData("finished", controller.finished);
            telemetry.update();

        }
    }
}
