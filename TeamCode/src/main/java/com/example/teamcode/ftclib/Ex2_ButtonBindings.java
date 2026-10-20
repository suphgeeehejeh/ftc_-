package com.example.teamcode.ftclib;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.StartEndCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.example.teamcode.ftclib.subsystems.KickerSubsystem;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * 예제 2: 버튼에 커맨드 연결하기
 *
 * [배우는 것]
 *   - whileHeld   : 누르고 있는 동안만 실행, 떼면 멈춤
 *   - whenPressed : 누른 순간 한 번 실행 (wasPressed 직접 안 짜도 됨)
 *   - InstantCommand  : 한 번 실행하고 바로 끝나는 커맨드
 *   - StartEndCommand : 시작할 때 A, 끝날 때 B를 실행하는 커맨드
 *   - run()을 덮어써서 telemetry 표시
 *
 * [조작] 킥 다리를 손으로 천천히 움직여 보는 테스트용
 *   A 누르고 있기 : 다리 천천히 올리기
 *   B 누르고 있기 : 다리 천천히 내리기
 *   Y 누르기      : 누른 횟수 +1 (InstantCommand 확인용)
 */
@TeleOp(name = "FTCLib 2 - Button Bindings", group = "FTCLib")
public class Ex2_ButtonBindings extends CommandOpMode {

    static final double JOG_DEG_S = 45.0; // 손으로 움직일 때 속도 (초당 각도)

    private KickerSubsystem kicker;
    private int yCount = 0;

    @Override
    public void initialize() {
        kicker = new KickerSubsystem(hardwareMap);
        GamepadEx driver = new GamepadEx(gamepad1);

        // 누르고 있는 동안: 시작할 때 속도 주고, 뗄 때(끝날 때) 멈춤
        driver.getGamepadButton(GamepadKeys.Button.A).whileHeld(new StartEndCommand(
                () -> kicker.setSpeedDegPerSec(JOG_DEG_S),
                kicker::stop,
                kicker));

        driver.getGamepadButton(GamepadKeys.Button.B).whileHeld(new StartEndCommand(
                () -> kicker.setSpeedDegPerSec(-JOG_DEG_S),
                kicker::stop,
                kicker));

        // 누른 순간 한 번. 아무 부품도 안 쓰니까 서브시스템을 안 넘김.
        driver.getGamepadButton(GamepadKeys.Button.Y).whenPressed(new InstantCommand(
                () -> yCount++));

        telemetry.addLine("Ready. A = leg up, B = leg down, Y = count.");
        telemetry.update();
    }

    /** CommandOpMode가 매 루프 부르는 함수. super.run()이 커맨드들을 돌림. */
    @Override
    public void run() {
        super.run();

        telemetry.addData("leg angle", "%.0f deg", kicker.getAngleDeg());
        telemetry.addData("kicker power", "%.2f", kicker.getPower());
        telemetry.addData("Y count", yCount);
        telemetry.update();
    }
}
