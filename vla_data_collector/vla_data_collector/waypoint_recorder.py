from __future__ import annotations

import json
import os
import tempfile
from datetime import datetime, timezone
from pathlib import Path
from typing import Optional

import numpy as np
import rclpy
from rclpy.node import Node

from .doosan_interface import DoosanInterface


DEFAULT_PLAN = [
    ("home", 1),
    ("approach", 1),
    ("pre_grasp", 1),
    ("grasp", 0),
    ("lift", 0),
    ("place", 0),
    ("release", 1),
    ("return_home", 1),
]


class WaypointRecorder(Node):
    """Hand-guiding 기반 Doosan TCP waypoint 기록기."""

    def __init__(self) -> None:
        super().__init__("waypoint_recorder", namespace="dsr01")

        self.declare_parameter("output_path", "waypoints/cube_pick.json")
        self.declare_parameter("instruction", "pick up the cube")
        self.declare_parameter("task", "cube_pick")
        self.declare_parameter("robot_id", "dsr01")
        self.declare_parameter("robot_model", "a0509")
        self.declare_parameter("pose_decimals", 6)
        self.declare_parameter("load_existing", False)

        self.output_path = Path(
            str(self.get_parameter("output_path").value)
        ).expanduser()
        self.instruction = str(
            self.get_parameter("instruction").value
        ).strip()
        self.task = str(self.get_parameter("task").value).strip()
        self.pose_decimals = int(
            self.get_parameter("pose_decimals").value
        )
        self.load_existing = bool(
            self.get_parameter("load_existing").value
        )

        if not self.instruction:
            raise ValueError("instruction must not be empty")
        if not self.task:
            raise ValueError("task must not be empty")
        if not 0 <= self.pose_decimals <= 10:
            raise ValueError("pose_decimals must be in 0..10")

        self.robot = DoosanInterface(
            node=self,
            robot_id=str(self.get_parameter("robot_id").value),
            robot_model=str(self.get_parameter("robot_model").value),
        )

        self.waypoints: list[dict] = []

        if self.load_existing and self.output_path.exists():
            self._load_existing_file()

    def _load_existing_file(self) -> None:
        data = json.loads(
            self.output_path.read_text(encoding="utf-8")
        )
        waypoints = data.get("waypoints")

        if not isinstance(waypoints, list):
            raise ValueError("waypoints must be a list")

        loaded: list[dict] = []
        names: set[str] = set()

        for index, waypoint in enumerate(waypoints):
            name = self._validate_name(waypoint.get("name"))
            if name in names:
                raise ValueError(
                    f"duplicate waypoint name: {name}"
                )

            pose = self._validate_pose(
                waypoint.get("tcp_pose"),
                context=f"waypoint {index}",
            )
            gripper = self._validate_gripper(
                waypoint.get("gripper")
            )

            loaded.append(
                {
                    "name": name,
                    "tcp_pose": pose,
                    "gripper": gripper,
                }
            )
            names.add(name)

        self.waypoints = loaded
        self.instruction = str(
            data.get("instruction", self.instruction)
        )
        self.task = str(data.get("task", self.task))

        self.get_logger().info(
            f"Loaded {len(self.waypoints)} waypoints: "
            f"{self.output_path}"
        )

    @staticmethod
    def _validate_name(value: object) -> str:
        if value is None:
            raise ValueError("waypoint name is missing")

        name = str(value).strip()
        if not name:
            raise ValueError("waypoint name must not be empty")

        invalid = ("/", "\\", "\n", "\r", "\t")
        if any(token in name for token in invalid):
            raise ValueError(
                "waypoint name contains an invalid character"
            )

        return name

    def _validate_pose(
        self,
        value: object,
        *,
        context: str = "waypoint",
    ) -> list[float]:
        pose = np.asarray(value, dtype=np.float64)

        if pose.shape != (6,):
            raise ValueError(
                f"{context}: tcp_pose must contain six values"
            )
        if not np.isfinite(pose).all():
            raise ValueError(
                f"{context}: tcp_pose contains NaN or infinity"
            )

        return (
            np.round(pose, self.pose_decimals)
            .astype(float)
            .tolist()
        )

    @staticmethod
    def _validate_gripper(value: object) -> int:
        try:
            gripper = int(value)
        except (TypeError, ValueError) as exc:
            raise ValueError("gripper must be 0 or 1") from exc

        if gripper not in (0, 1):
            raise ValueError(
                "gripper must be 0 (closed) or 1 (open)"
            )

        return gripper

    def _find_index(self, name: str) -> Optional[int]:
        for index, waypoint in enumerate(self.waypoints):
            if waypoint["name"] == name:
                return index
        return None

    def capture_current_pose(self) -> list[float]:
        pose = self.robot.get_tcp_pose().as_array()
        return self._validate_pose(
            pose,
            context="current TCP pose",
        )

    def add_or_replace_waypoint(
        self,
        name: str,
        gripper: int,
        *,
        replace: bool = False,
    ) -> dict:
        name = self._validate_name(name)
        gripper = self._validate_gripper(gripper)

        existing_index = self._find_index(name)
        if existing_index is not None and not replace:
            raise ValueError(
                f"Waypoint already exists: {name}"
            )

        waypoint = {
            "name": name,
            "tcp_pose": self.capture_current_pose(),
            "gripper": gripper,
        }

        if existing_index is None:
            self.waypoints.append(waypoint)
        else:
            self.waypoints[existing_index] = waypoint

        return waypoint

    def delete_waypoint(self, name: str) -> bool:
        index = self._find_index(self._validate_name(name))
        if index is None:
            return False

        del self.waypoints[index]
        return True

    def move_waypoint(
        self,
        old_index: int,
        new_index: int,
    ) -> None:
        count = len(self.waypoints)

        if not 0 <= old_index < count:
            raise IndexError("old waypoint index out of range")
        if not 0 <= new_index < count:
            raise IndexError("new waypoint index out of range")

        waypoint = self.waypoints.pop(old_index)
        self.waypoints.insert(new_index, waypoint)

    def preview(self) -> None:
        print("\n" + "=" * 96)
        print(
            f"{'#':>3}  {'name':<18}  {'gripper':<10}  "
            "x[m]       y[m]       z[m]       "
            "rx[rad]    ry[rad]    rz[rad]"
        )
        print("-" * 96)

        if not self.waypoints:
            print("(저장된 waypoint 없음)")
        else:
            for index, waypoint in enumerate(
                self.waypoints,
                start=1,
            ):
                pose = waypoint["tcp_pose"]
                state = (
                    "0 closed"
                    if waypoint["gripper"] == 0
                    else "1 open"
                )
                pose_text = "  ".join(
                    f"{value:>9.{self.pose_decimals}f}"
                    for value in pose
                )
                print(
                    f"{index:>3}  "
                    f"{waypoint['name']:<18}  "
                    f"{state:<10}  "
                    f"{pose_text}"
                )

        print("=" * 96)
        print(f"instruction : {self.instruction}")
        print(f"task        : {self.task}")
        print(f"output      : {self.output_path}\n")

    def _build_output_data(self) -> dict:
        if not self.waypoints:
            raise ValueError(
                "Cannot save an empty waypoint list"
            )

        names = [
            waypoint["name"]
            for waypoint in self.waypoints
        ]

        if len(names) != len(set(names)):
            raise ValueError(
                "Duplicate waypoint names detected"
            )

        return {
            "instruction": self.instruction,
            "task": self.task,
            "robot": "Doosan A0509",
            "camera": "Stereolabs ZED 2i",
            "camera_mount": "eye_in_hand_vertical",
            "coordinate_frame": "base",
            "position_unit": "meter",
            "rotation_unit": "radian",
            "gripper_convention": {
                "0": "closed",
                "1": "open",
            },
            "waypoint_gripper_semantics": (
                "desired gripper state after arriving "
                "at the waypoint"
            ),
            "created_at_utc": (
                datetime.now(timezone.utc).isoformat()
            ),
            "waypoints": self.waypoints,
        }

    def save(
        self,
        *,
        overwrite: bool = False,
    ) -> None:
        data = self._build_output_data()
        self.output_path.parent.mkdir(
            parents=True,
            exist_ok=True,
        )

        if self.output_path.exists() and not overwrite:
            raise FileExistsError(
                f"Output already exists: {self.output_path}"
            )

        fd, temporary_name = tempfile.mkstemp(
            prefix=f".{self.output_path.name}.",
            suffix=".tmp",
            dir=str(self.output_path.parent),
            text=True,
        )

        try:
            with os.fdopen(
                fd,
                "w",
                encoding="utf-8",
            ) as file:
                json.dump(
                    data,
                    file,
                    ensure_ascii=False,
                    indent=2,
                )
                file.write("\n")
                file.flush()
                os.fsync(file.fileno())

            os.replace(
                temporary_name,
                self.output_path,
            )

        except Exception:
            Path(temporary_name).unlink(
                missing_ok=True
            )
            raise

        self.get_logger().info(
            f"Saved {len(self.waypoints)} waypoints: "
            f"{self.output_path}"
        )


def ask_yes_no(
    prompt: str,
    *,
    default: bool = False,
) -> bool:
    suffix = " [Y/n]: " if default else " [y/N]: "

    while True:
        answer = input(prompt + suffix).strip().lower()

        if not answer:
            return default
        if answer in ("y", "yes"):
            return True
        if answer in ("n", "no"):
            return False

        print("y 또는 n을 입력하십시오.")


def ask_gripper(
    default: Optional[int] = None,
) -> int:
    while True:
        default_text = (
            ""
            if default is None
            else f" [기본값 {default}]"
        )

        answer = input(
            "도착 후 그리퍼 상태 "
            "(0=닫기, 1=열기)"
            f"{default_text}: "
        ).strip()

        if not answer and default is not None:
            return default
        if answer in ("0", "1"):
            return int(answer)

        print("0 또는 1을 입력하십시오.")


def ask_waypoint_number(
    node: WaypointRecorder,
    prompt: str,
) -> Optional[int]:
    if not node.waypoints:
        print("선택할 waypoint가 없습니다.")
        return None

    node.preview()
    answer = input(prompt).strip()

    try:
        number = int(answer)
    except ValueError:
        print("숫자를 입력하십시오.")
        return None

    if not 1 <= number <= len(node.waypoints):
        print("범위를 벗어난 번호입니다.")
        return None

    return number - 1


def capture_interactively(
    node: WaypointRecorder,
    name: str,
    default_gripper: Optional[int] = None,
) -> None:
    name = name.strip()

    if not name:
        print("이름은 비워둘 수 없습니다.")
        return

    existing_index = node._find_index(name)
    replace = False

    if existing_index is not None:
        replace = ask_yes_no(
            f"'{name}' waypoint가 이미 있습니다. "
            "현재 TCP pose로 교체하시겠습니까?"
        )
        if not replace:
            return

    gripper = ask_gripper(default_gripper)

    print(
        f"\n로봇을 '{name}' 위치로 Hand Guiding한 뒤 "
        "Enter를 누르십시오.\n"
        "그리퍼 값은 현재 실제 출력이 아니라, "
        "해당 위치 도착 후 적용할 목표 상태입니다."
    )
    input("준비되면 Enter: ")

    waypoint = node.add_or_replace_waypoint(
        name,
        gripper,
        replace=replace,
    )

    pose = waypoint["tcp_pose"]

    print(
        "\n캡처 결과\n"
        f"  name    : {waypoint['name']}\n"
        f"  pose    : {pose}\n"
        f"  gripper : {waypoint['gripper']} "
        f"({'closed' if waypoint['gripper'] == 0 else 'open'})"
    )

    if not ask_yes_no(
        "이 값을 유지하시겠습니까?",
        default=True,
    ):
        node.delete_waypoint(name)
        print("방금 캡처한 waypoint를 취소했습니다.")


def record_default_plan(
    node: WaypointRecorder,
) -> None:
    for name, gripper in DEFAULT_PLAN:
        print(f"\n[{name}] waypoint 기록")
        capture_interactively(
            node,
            name,
            gripper,
        )

        if not ask_yes_no(
            "다음 waypoint로 진행하시겠습니까?",
            default=True,
        ):
            break


def edit_menu(
    node: WaypointRecorder,
) -> None:
    while rclpy.ok():
        print(
            "\n명령을 선택하십시오.\n"
            "  1. waypoint 추가\n"
            "  2. waypoint 다시 기록\n"
            "  3. waypoint 삭제\n"
            "  4. waypoint 순서 변경\n"
            "  5. 목록 미리보기\n"
            "  6. instruction/task 수정\n"
            "  7. 저장 후 종료\n"
            "  8. 저장하지 않고 종료\n"
        )

        command = input("선택: ").strip()

        try:
            if command == "1":
                name = input(
                    "새 waypoint 이름: "
                ).strip()
                capture_interactively(
                    node,
                    name,
                )

            elif command == "2":
                index = ask_waypoint_number(
                    node,
                    "다시 기록할 waypoint 번호: ",
                )
                if index is None:
                    continue

                current = node.waypoints[index]
                capture_interactively(
                    node,
                    current["name"],
                    int(current["gripper"]),
                )

            elif command == "3":
                index = ask_waypoint_number(
                    node,
                    "삭제할 waypoint 번호: ",
                )
                if index is None:
                    continue

                name = node.waypoints[index]["name"]

                if ask_yes_no(
                    f"'{name}' waypoint를 "
                    "삭제하시겠습니까?"
                ):
                    node.delete_waypoint(name)
                    print("삭제했습니다.")

            elif command == "4":
                old_index = ask_waypoint_number(
                    node,
                    "이동할 waypoint 번호: ",
                )
                if old_index is None:
                    continue

                answer = input(
                    f"새 위치 번호 "
                    f"(1~{len(node.waypoints)}): "
                ).strip()

                try:
                    new_index = int(answer) - 1
                    node.move_waypoint(
                        old_index,
                        new_index,
                    )
                except (ValueError, IndexError) as exc:
                    print(f"순서 변경 실패: {exc}")

            elif command == "5":
                node.preview()

            elif command == "6":
                instruction = input(
                    f"instruction "
                    f"[{node.instruction}]: "
                ).strip()
                task = input(
                    f"task [{node.task}]: "
                ).strip()

                if instruction:
                    node.instruction = instruction
                if task:
                    node.task = task

            elif command == "7":
                node.preview()

                if not ask_yes_no(
                    "이 순서와 값으로 "
                    "저장하시겠습니까?"
                ):
                    continue

                overwrite = False

                if node.output_path.exists():
                    overwrite = ask_yes_no(
                        f"{node.output_path} 파일이 "
                        "이미 있습니다. 덮어쓰시겠습니까?"
                    )
                    if not overwrite:
                        continue

                node.save(overwrite=overwrite)
                return

            elif command == "8":
                if ask_yes_no(
                    "기록 내용을 저장하지 않고 "
                    "종료하시겠습니까?"
                ):
                    return

            else:
                print("1~8 중 하나를 입력하십시오.")

        except Exception as exc:
            node.get_logger().error(str(exc))


def main(args=None) -> None:
    rclpy.init(args=args)
    node: Optional[WaypointRecorder] = None

    try:
        node = WaypointRecorder()

        print(
            "\n=== Doosan A0509 Waypoint Recorder ===\n"
            "로봇 컨트롤러에서 Hand Guiding이 가능한 "
            "안전 상태인지 먼저 확인하십시오.\n"
            "이 노드는 TCP pose만 읽고 자동 이동하지 않습니다.\n"
            "gripper 값은 waypoint 도착 후 적용할 "
            "목표 상태입니다.\n"
        )

        if (
            not node.waypoints
            and ask_yes_no(
                "기본 cube-pick 순서로 "
                "기록을 시작하시겠습니까?",
                default=True,
            )
        ):
            record_default_plan(node)

        edit_menu(node)

    except KeyboardInterrupt:
        if node is not None:
            node.get_logger().warning(
                "Waypoint recording interrupted"
            )
    except Exception as exc:
        if node is not None:
            node.get_logger().error(
                f"Waypoint recorder failed: {exc}"
            )
        raise
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
