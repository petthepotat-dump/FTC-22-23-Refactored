package org.firstinspires.ftc.teamcode.wrappers;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class ArmController {
    private PIDController controller;
    public static double p = 0, i = 0, d = 0, f = 0;
    public static int target = 0;
    private DcMotorEx leftArm, rightArm;

    public ArmController() {
        controller = new PIDController(p, i, d);
        controller.setTolerance(0.1);

    }
}
