package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class SimpleLocalizer {
    private DcMotorEx podForward, podStrafe;
    private IMU imu;

    private final double INCHES_PER_TICK = (1.88976 * Math.PI) / 2000.0;

    public double x = 0, y = 0, heading = 0;
    private double lastForward = 0, lastStrafe = 0;

    public SimpleLocalizer(HardwareMap hardwareMap) {
        podForward = hardwareMap.get(DcMotorEx.class, "par0");
        podStrafe = hardwareMap.get(DcMotorEx.class, "par1");
        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(logoDirection, usbDirection)));
        reset();
    }

    public void reset() {
        imu.resetYaw();
        lastForward = podForward.getCurrentPosition() * INCHES_PER_TICK;
        lastStrafe = podStrafe.getCurrentPosition() * INCHES_PER_TICK;
        x = 0;
        y = 0;
        heading = 0;
    }

    public void update() {
        double curForward = podForward.getCurrentPosition() * INCHES_PER_TICK;
        double curStrafe = podStrafe.getCurrentPosition() * INCHES_PER_TICK;

        double dForward = curForward - lastForward;
        double dStrafe = curStrafe - lastStrafe;

        heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        double dX = dStrafe * Math.cos(heading) + dForward * Math.sin(heading);
        double dY = -dStrafe * Math.sin(heading) + dForward * Math.cos(heading);

        x += dX;
        y += dY;

        lastForward = curForward;
        lastStrafe = curStrafe;
    }
}