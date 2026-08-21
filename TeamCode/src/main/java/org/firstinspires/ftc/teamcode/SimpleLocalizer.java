package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public class SimpleLocalizer {
    public GoBildaPinpointDriver odo;

    public double x = 0, y = 0, heading = 0;

    public SimpleLocalizer (HardwareMap hardwareMap) {
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
        odo.resetPosAndIMU(); // 強制歸零
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
}