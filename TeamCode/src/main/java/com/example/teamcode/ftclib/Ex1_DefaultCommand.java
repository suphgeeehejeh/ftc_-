package com.example.teamcode.ftclib;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.example.teamcode.ftclib.subsystems.DriveSubsystem;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * 예제 1: 가장 기본 - 기본 커맨드(Default Command)로 주행
 *
 * [배우는 것]
 *   - CommandOpMode: while 루프를 직접 안 씀. initialize()에서 "설정"만 하면 끝.
 *   - 서브시스템: 로봇 부품 (여기선 DriveSubsystem)
 *   - setDefaultCommand: 그 부품이 할 일이 없을 때 계속 하는 일
 *
 * [조작] 왼쪽 스틱 = 이동, 오른쪽 스틱 좌우 = 회전
 *
 * mecanum_drive.java의 while 루프 안에서 하던 일을 한 줄짜리 RunCommand가 대신함.
 */
@TeleOp(name = "FTCLib 1 - Default Command", group = "FTCLib")
public class Ex1_DefaultCommand extends CommandOpMode {

    static final double SPEED = 0.5;

    @Override
    public void initialize() {
        DriveSubsystem drive = new DriveSubsystem(hardwareMap);
        GamepadEx driver = new GamepadEx(gamepad1); // getLeftY()는 위로 밀면 +

        // RunCommand = 매 루프 이 코드를 실행. 끝나지 않음.
        // 뒤에 drive를 넘겨서 "이 커맨드는 drive 부품을 씀"이라고 알려 줌.
        drive.setDefaultCommand(new RunCommand(
                () -> drive.drive(driver.getLeftX(), driver.getLeftY(), driver.getRightX(), SPEED),
                drive));

        telemetry.addLine("Ready. Left stick = move, right stick = turn.");
        telemetry.update();
    }
}
