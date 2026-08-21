package org.firstinspires.ftc.teamcode;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;


@TeleOp(name="TEAMOP")
public abstract class BIOBUZZ_SEASON extends LinearOpMode {
    DcMotor BL,BR,FL,FR;
    public void init_hardware(){
        BL=hardwareMap.get(DcMotor.class,"BL");
        BR=hardwareMap.get(DcMotor.class,"BR");
        FL=hardwareMap.get(DcMotor.class,"FL");
        FR=hardwareMap.get(DcMotor.class,"FR");
        BL.setDirection(DcMotor.Direction.REVERSE);
        BR.setDirection(DcMotor.Direction.FORWARD);
        FL.setDirection(DcMotor.Direction.REVERSE);
        FR.setDirection(DcMotor.Direction.FORWARD);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    @Override
    public void runOpMode(){
        init_hardware();
        SimpleLocalizer localizer = new SimpleLocalizer(hardwareMap);
        localizer.reset();
        telemetry.addData("狀態", "擺正車頭後按 Play");
        telemetry.update();
        waitForStart();
        while(opModeIsActive()){
            localizer.update();
            if (gamepad1.startWasPressed()) {
                localizer.reset();
            }
            double botHeading = localizer.heading;

            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            //AI幫的啦，誰懂三角函數
            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);


            BL.setPower(rotY + rotX + rx);
            BR.setPower(rotY - rotX - rx);
            FL.setPower(rotY - rotX + rx);
            FR.setPower(rotY + rotX - rx);

            // Driver Station 即時監控
            telemetry.addData("絕對座標 X (公分)", "%.2f", (localizer.x)*2.54);
            telemetry.addData("絕對座標 Y (公分)", "%.2f", (localizer.y)*2.54);
            telemetry.addData("絕對角度 (度)", "%.2f", Math.toDegrees(botHeading));
            telemetry.update();
            //絕對底盤結束
        }
    }
}
