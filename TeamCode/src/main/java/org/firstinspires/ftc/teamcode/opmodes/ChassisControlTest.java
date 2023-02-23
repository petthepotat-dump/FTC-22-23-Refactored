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
    public volatile static double x, y, theta;
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
        controller.goTo(x, y, theta, 0, 0, 1, 1, true);
        while (!controller.finished) {
            if (isStopRequested()) controller.stop();
        }
    }
}
