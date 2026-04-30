#!/usr/bin/env python3
import time
from dataclasses import dataclass, field
from typing import Optional


HEARTBEAT_TIMEOUT = 0.5
UR_TASK_TIMEOUT = 20.0
KUKA_TASK_TIMEOUT = 20.0
HOMING_TIMEOUT = 30.0
READY_TIMEOUT = 10.0
RECOVERY_TIMEOUT = 30.0


@dataclass
class RobotStatus:
    name: str
    state: str = "offline"
    task: str = "none"
    error: bool = False
    heartbeat_time: float = 0.0
    last_event: str = ""
    ready: bool = False
    homed: bool = False


@dataclass
class SupervisorContext:
    current_state: str = "INIT"
    previous_state: str = "INIT"
    cycle_id: int = 0
    auto_mode: bool = True
    emergency_active: bool = False
    paused: bool = False
    state_enter_time: float = field(default_factory=time.time)
    ur: RobotStatus = field(default_factory=lambda: RobotStatus(name="ur5e"))
    kuka: RobotStatus = field(default_factory=lambda: RobotStatus(name="kuka"))


class SupervisorFSM:
    def __init__(self):
        self.ctx = SupervisorContext()
        self.start_requested = False
        self.reset_requested = False
        self.pause_requested = False
        self.resume_requested = False

    def now(self) -> float:
        return time.time()

    def time_in_state(self) -> float:
        return self.now() - self.ctx.state_enter_time

    def transition(self, new_state: str) -> None:
        print(f"[FSM] {self.ctx.current_state} -> {new_state}")
        self.ctx.previous_state = self.ctx.current_state
        self.ctx.current_state = new_state
        self.ctx.state_enter_time = self.now()

    def is_alive(self, robot: RobotStatus) -> bool:
        return (self.now() - robot.heartbeat_time) <= HEARTBEAT_TIMEOUT

    def both_alive(self) -> bool:
        return self.is_alive(self.ctx.ur) and self.is_alive(self.ctx.kuka)

    def both_ready(self) -> bool:
        return (
            self.ctx.ur.ready and
            self.ctx.kuka.ready and
            self.ctx.ur.state == "idle" and
            self.ctx.kuka.state == "idle" and
            not self.ctx.ur.error and
            not self.ctx.kuka.error
        )

    def publish_command(self, target: str, command: str, job_id: Optional[int] = None) -> None:
        print(f"[CMD] target={target}, command={command}, job_id={job_id}")

    def latch_error(self, message: str) -> None:
        print(f"[ERROR] {message}")
        self.transition("ERROR")

    def latch_emergency(self, message: str) -> None:
        print(f"[EMERGENCY] {message}")
        self.ctx.emergency_active = True
        self.transition("EMERGENCY_STOP")

    def clear_requests(self) -> None:
        self.start_requested = False
        self.reset_requested = False
        self.pause_requested = False
        self.resume_requested = False

    # -----------------------------
    # External event update methods
    # -----------------------------
    def update_robot_state(self, robot_name: str, state: str, task: str, error: bool, ready: bool, homed: bool) -> None:
        robot = self.ctx.ur if robot_name == "ur5e" else self.ctx.kuka
        robot.state = state
        robot.task = task
        robot.error = error
        robot.ready = ready
        robot.homed = homed
        robot.heartbeat_time = self.now()

    def update_robot_event(self, robot_name: str, event: str) -> None:
        robot = self.ctx.ur if robot_name == "ur5e" else self.ctx.kuka
        robot.last_event = event
        robot.heartbeat_time = self.now()

    def set_emergency(self, active: bool) -> None:
        self.ctx.emergency_active = active
        if active:
            self.latch_emergency("External emergency signal received")

    def request_start(self) -> None:
        self.start_requested = True

    def request_pause(self) -> None:
        self.pause_requested = True

    def request_resume(self) -> None:
        self.resume_requested = True

    def request_reset(self) -> None:
        self.reset_requested = True

    # -----------------------------
    # Main FSM update
    # -----------------------------
    def step(self) -> None:
        # Global high-priority logic
        if self.ctx.emergency_active and self.ctx.current_state != "EMERGENCY_STOP":
            self.latch_emergency("Emergency active")
            return

        if self.ctx.current_state not in ("INIT", "EMERGENCY_STOP") and not self.both_alive():
            self.latch_error("Heartbeat lost from one or both robots")
            return

        if self.ctx.current_state not in ("ERROR", "EMERGENCY_STOP") and (self.ctx.ur.error or self.ctx.kuka.error):
            self.latch_error("Robot error detected")
            return

        if self.pause_requested and self.ctx.current_state not in ("PAUSED", "ERROR", "EMERGENCY_STOP"):
            self.transition("PAUSED")
            self.pause_requested = False
            return

        # State-specific logic
        state = self.ctx.current_state

        if state == "INIT":
            if self.both_alive():
                self.transition("HOMING")

        elif state == "HOMING":
            if self.time_in_state() == 0:
                pass
            if self.ctx.ur.homed and self.ctx.kuka.homed:
                self.transition("IDLE")
            elif self.time_in_state() > HOMING_TIMEOUT:
                self.latch_error("Homing timeout")

        elif state == "IDLE":
            if self.start_requested:
                self.start_requested = False
                self.transition("WAIT_BOTH_READY")

        elif state == "WAIT_BOTH_READY":
            if self.both_ready():
                self.transition("START_UR_TASK")
            elif self.time_in_state() > READY_TIMEOUT:
                self.latch_error("Robots not ready in time")

        elif state == "START_UR_TASK":
            self.ctx.cycle_id += 1
            self.publish_command("ur5e", "START_PICK_AND_PLACE_TO_HANDOVER", self.ctx.cycle_id)
            self.transition("WAIT_UR_DONE")

        elif state == "WAIT_UR_DONE":
            if self.ctx.ur.last_event == "TASK_DONE":
                self.ctx.ur.last_event = ""
                self.transition("START_KUKA_TASK")
            elif self.time_in_state() > UR_TASK_TIMEOUT:
                self.latch_error("UR task timeout")

        elif state == "START_KUKA_TASK":
            self.publish_command("kuka", "START_PICK_AND_SORT", self.ctx.cycle_id)
            self.transition("WAIT_KUKA_DONE")

        elif state == "WAIT_KUKA_DONE":
            if self.ctx.kuka.last_event == "TASK_DONE":
                self.ctx.kuka.last_event = ""
                self.transition("CYCLE_COMPLETE")
            elif self.time_in_state() > KUKA_TASK_TIMEOUT:
                self.latch_error("KUKA task timeout")

        elif state == "CYCLE_COMPLETE":
            print(f"[INFO] Cycle {self.ctx.cycle_id} completed successfully")
            if self.ctx.auto_mode:
                self.transition("WAIT_BOTH_READY")
            else:
                self.transition("IDLE")

        elif state == "PAUSED":
            if self.resume_requested:
                self.resume_requested = False
                self.transition("WAIT_BOTH_READY")

        elif state == "ERROR":
            if self.reset_requested:
                self.reset_requested = False
                self.transition("RECOVERY")

        elif state == "EMERGENCY_STOP":
            if not self.ctx.emergency_active and self.reset_requested:
                self.reset_requested = False
                self.transition("RECOVERY")

        elif state == "RECOVERY":
            if self.both_alive() and not self.ctx.ur.error and not self.ctx.kuka.error:
                if self.time_in_state() < RECOVERY_TIMEOUT:
                    self.transition("IDLE")
                else:
                    self.latch_error("Recovery timeout")

        else:
            self.latch_error(f"Unknown state: {state}")


def demo():
    fsm = SupervisorFSM()

    # Simulate both robots becoming alive
    while fsm.ctx.current_state != "IDLE":
        fsm.update_robot_state("ur5e", "idle", "none", False, True, True)
        fsm.update_robot_state("kuka", "idle", "none", False, True, True)
        fsm.step()
        time.sleep(0.1)

    # Start cycle
    fsm.request_start()

    for _ in range(5):
        fsm.update_robot_state("ur5e", "idle", "none", False, True, True)
        fsm.update_robot_state("kuka", "idle", "none", False, True, True)
        fsm.step()
        time.sleep(0.1)

    # Simulate UR done
    fsm.update_robot_event("ur5e", "TASK_DONE")
    for _ in range(5):
        fsm.update_robot_state("ur5e", "idle", "none", False, True, True)
        fsm.update_robot_state("kuka", "idle", "none", False, True, True)
        fsm.step()
        time.sleep(0.1)

    # Simulate KUKA done
    fsm.update_robot_event("kuka", "TASK_DONE")
    for _ in range(5):
        fsm.update_robot_state("ur5e", "idle", "none", False, True, True)
        fsm.update_robot_state("kuka", "idle", "none", False, True, True)
        fsm.step()
        time.sleep(0.1)


if __name__ == "__main__":
    demo()
