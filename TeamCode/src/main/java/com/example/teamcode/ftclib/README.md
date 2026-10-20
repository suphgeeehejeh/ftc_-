# FTCLib 커맨드 기반 예제

`mecanum_drive.java`와 같은 로봇(메카넘 바퀴 `motor1`~`motor4`, Expansion Hub의 `kicker`)으로 돌아가요.
Driver Station에서 OpMode 목록의 **FTCLib** 그룹에 있어요. 1번부터 순서대로 보면 돼요.

| 파일 | 배우는 것 |
|---|---|
| `Ex1_DefaultCommand` | `CommandOpMode`, 서브시스템, 기본 커맨드로 주행 |
| `Ex2_ButtonBindings` | `whileHeld` / `whenPressed`, `InstantCommand`, `StartEndCommand` |
| `Ex3_KickSequence` | 직접 만든 커맨드, `SequentialCommandGroup`, `WaitCommand` 쿨다운 |
| `Ex4_Autonomous` | 같은 커맨드로 자율주행, `withTimeout`, `WaitUntilCommand` |

## 폴더 구조
```
ftclib/
├── subsystems/          로봇 부품: "무엇을 할 수 있는지"
│   ├── DriveSubsystem   메카넘 바퀴 4개
│   └── KickerSubsystem  킥 다리
├── commands/            직접 만든 커맨드: "무엇을 할지"
│   ├── KickCommand      정해진 각도까지 휘두르기
│   └── LowerLegCommand  천천히 내리기
└── Ex1 ~ Ex4            OpMode: 버튼과 커맨드를 연결
```

## 한 줄 요약
- **서브시스템** = 부품. 모터를 가지고 있고 `drive()`, `stop()` 같은 기능만 있어요.
- **커맨드** = 할 일. `initialize` → `execute`(반복) → `isFinished`가 true → `end` 순서로 실행돼요.
- **CommandOpMode** = `while` 루프를 직접 안 써요. `initialize()`에서 버튼과 커맨드를 연결해 두면 FTCLib이 알아서 돌려요.
