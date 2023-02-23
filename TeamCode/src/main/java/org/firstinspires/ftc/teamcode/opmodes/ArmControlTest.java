package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.wrappers.MecanumChassis;
import org.firstinspires.ftc.teamcode.wrappers.Position;

@Config
@TeleOp(name="ArmConfig")

public class ArmControlTest extends LinearOpMode {
    private MecanumChassis robot;
    private Position pos;
    private PIDController controller;
    private FtcDashboard dashboard;
    public volatile static double p = 0.0015, i = 0.0007, d = 0.00012, f = 0;
    public volatile static int target = 0;
    @Override
    public void runOpMode() {
        dashboard = FtcDashboard.getInstance();
        controller = new PIDController(p, i, d);
        robot = new MecanumChassis(hardwareMap);
        pos = new Position(robot);
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        robot.rightArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        dashboard.updateConfig();
        waitForStart();
        while (!isStopRequested()) {
            int armPos = robot.leftArm.getCurrentPosition();
            controller.setPID(p, i, d);
            double pid = controller.calculate(armPos, target);
            double power = pid;
            robot.rightArm.setPower(power);
            robot.leftArm.setPower(power);
            telemetry.addData("pos", armPos);
            telemetry.addData("target", target);
            telemetry.addData("power", power);
            telemetry.update();
        }
    }
}
