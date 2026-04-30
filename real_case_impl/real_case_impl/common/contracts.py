import json
from typing import Any


def dumps(payload: dict[str, Any]) -> str:
    return json.dumps(payload)


def loads(raw: str) -> dict[str, Any]:
    value = json.loads(raw)
    if not isinstance(value, dict):
        raise ValueError("JSON payload must be an object")
    return value


def task_command_payload(command: str, job_id: int, timestamp: float, source: str = "") -> dict[str, Any]:
    payload: dict[str, Any] = {
        "command": command,
        "job_id": job_id,
        "timestamp": timestamp,
    }
    if source:
        payload["source"] = source
    return payload


def task_status_payload(
    robot: str,
    state: str,
    task: str,
    ready: bool,
    homed: bool,
    motion: str,
    error: bool,
    message: str,
    timestamp: float,
    job_id: int | None = None,
) -> dict[str, Any]:
    payload: dict[str, Any] = {
        "robot": robot,
        "state": state,
        "task": task,
        "ready": ready,
        "homed": homed,
        "motion": motion,
        "error": error,
        "message": message,
        "timestamp": timestamp,
    }
    if job_id is not None:
        payload["job_id"] = job_id
    return payload


def task_event_payload(
    robot: str,
    event: str,
    task: str,
    job_id: int,
    timestamp: float,
    message: str = "",
) -> dict[str, Any]:
    return {
        "robot": robot,
        "event": event,
        "task": task,
        "job_id": job_id,
        "message": message,
        "timestamp": timestamp,
    }

