package org.firstinspires.ftc.teamcode.TeleOp_Test;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;


@TeleOp(name="TEAMOP")
public class Pinpoint_Test extends LinearOpMode {
    DcMotor BL,BR,FL,FR;
    public GoBildaPinpointDriver odo;

    public double x = 0, y = 0, heading = 0;

    public void init_pinpoint() {
        // 讀取在 Robot Configuration 裡設定名稱為 "pinpoint" 的裝置
        odo = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        // 1. 設定定位輪類型 (請依實際購買的 Pod 型號選擇，例如 4-Bar 或 Swingarm)
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);

        // 2. 設定 Pod 距離機器人旋轉中心的物理距離 (單位：公釐 mm)
        odo.setOffsets(-84.0, -168.0, DistanceUnit.MM);
        // 3. 設定 Encoder 正反向 (若推動機器人時座標變負數可修改此處)
        odo.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD
        );

        // 4. 重設座標與角度
        reset();
    }

    public void reset() {
        odo.resetPosAndIMU();// 強制歸零
        sleep(300);//保險
    }

    public void update() {
        // 觸發 Pinpoint 讀取與計算
        odo.update();

        // 提取計算好的場地絕對座標
        Pose2D pose = odo.getPosition();
        x = pose.getX(DistanceUnit.INCH);      // 轉換為 吋
        y = pose.getY(DistanceUnit.INCH);      // 轉換為 吋
        heading = pose.getHeading(AngleUnit.RADIANS); // 取得弧度角
    }
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
        init_pinpoint();
        init_hardware();
        telemetry.addData("狀態", "擺正車頭後按 Play");
        telemetry.update();
        waitForStart();
        while(opModeIsActive()){
            update();
            if (gamepad1.startWasPressed()) {
                reset();
            }
            double botHeading = heading;
            double Hy = -gamepad1.left_stick_y;
            double Hx = gamepad1.left_stick_x;
            double Hrx = gamepad1.right_stick_x;

            //AI幫的啦，誰懂三角函數
            double rotX = Hx * Math.cos(-botHeading) - Hy * Math.sin(-botHeading);
            double rotY = Hx * Math.sin(-botHeading) + Hy * Math.cos(-botHeading);

            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(Hrx), 1.0);
            FL.setPower((rotY + rotX + Hrx) / denominator);
            FR.setPower((rotY - rotX - Hrx) / denominator);
            BL.setPower((rotY - rotX + Hrx) / denominator);
            BR.setPower((rotY + rotX - Hrx) / denominator);

            // Driver Station 即時監控
            telemetry.addData("絕對座標 X (公分)", "%.2f", (x)*2.54);
            telemetry.addData("絕對座標 Y (公分)", "%.2f", (y)*2.54);
            telemetry.addData("絕對角度", "%.2f", Math.toDegrees(botHeading));
            telemetry.update();
            //絕對底盤結束

        }
    }
}
