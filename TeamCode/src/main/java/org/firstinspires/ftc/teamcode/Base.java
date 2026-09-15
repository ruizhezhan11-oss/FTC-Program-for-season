package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp
public class Base extends LinearOpMode {
    private DcMotor FL,FR,BL,BR;
    private GoBildaPinpointDriver pinpoint;
    private void init_(){
        BL = hardwareMap.get(DcMotor.class, "BL");
        BR = hardwareMap.get(DcMotor.class, "BR");
        FL = hardwareMap.get(DcMotor.class, "FL");
        FR = hardwareMap.get(DcMotor.class, "FR");

        BL.setDirection(DcMotor.Direction.REVERSE);
        BR.setDirection(DcMotor.Direction.FORWARD);
        FL.setDirection(DcMotor.Direction.REVERSE);
        FR.setDirection(DcMotor.Direction.FORWARD);

        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);




        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setOffsets(-84.0, -168.0, DistanceUnit.MM);
        pinpoint.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD
        );
        pinpoint.resetPosAndIMU();

    }
    @Override
    public  void runOpMode(){
        waitForStart();
        while (opModeIsActive()){
        pinpoint.update();
        run_code();
        word_screen();
        telemetry.update();

        }
    }
    private void run_code(){
        Pose2D pose2D = pinpoint.getPosition();
        double y=applyDeadzone(gamepad1.left_stick_y);
        double x=applyDeadzone(gamepad1.left_stick_x);
        double rx=applyDeadzone(gamepad1.right_stick_x);
        double Heading = pose2D.getHeading(AngleUnit.RADIANS);
        double rotX = x * Math.cos(-Heading) - y * Math.sin(-Heading);
        double rotY = x * Math.sin(-Heading) + y * Math.cos(-Heading);
        double flPower = rotY + rotX + rx;
        double frPower = rotY - rotX - rx;
        double blPower = rotY - rotX + rx;
        double brPower = rotY + rotX - rx;
        FL.setPower(flPower);
        FR.setPower(frPower);
        BL.setPower(blPower);
        BR.setPower(brPower);

    }
    public void word_screen(){
        telemetry.addData("Heading",pinpoint.getHeading(AngleUnit.DEGREES));
        telemetry.addData("X",pinpoint.getEncoderX());
        telemetry.addData("Y",pinpoint.getEncoderY());
    }
    private double applyDeadzone(double input) {
        return Math.abs(input) > 0.05 ? input : 0.0;
    }
}
