DEFAULT_UR_ROBOT_NAME = "ur5e"
DEFAULT_KUKA_ROBOT_NAME = "kuka"

COMMAND_HOME = "HOME"
COMMAND_UR_TASK = "START_PICK_AND_PLACE_TO_HANDOVER"
COMMAND_KUKA_TASK = "START_PICK_AND_SORT"
COMMAND_STOP = "STOP"
COMMAND_RESET = "RESET"

CELL_OPERATOR_CMD_TOPIC = "/cell/operator_cmd"
CELL_SUPERVISOR_STATUS_TOPIC = "/cell/supervisor_status"
CELL_VISUAL_SAFETY_STATUS_TOPIC = "/cell/visual_safety_status"
CELL_VISUAL_SAFETY_MOSAIC_TOPIC = "/cell/visual_safety_mosaic"

CELL_SUPERVISOR_STATE_MQTT_TOPIC = "cell/supervisor/state"
CELL_SAFETY_ALARM_MQTT_TOPIC = "cell/safety/alarm"
CELL_SAFETY_RESET_MQTT_TOPIC = "cell/safety/reset"
CELL_OPERATOR_CMD_MQTT_TOPIC = "cell/operator/cmd"

KUKA_BACKEND_CMD_MQTT_TOPIC = "backend/kuka/cmd"
KUKA_BACKEND_STATUS_MQTT_TOPIC = "backend/kuka/status"
KUKA_BACKEND_EVENT_MQTT_TOPIC = "backend/kuka/event"


def robot_task_cmd_topic(robot_name: str) -> str:
    return f"/{robot_name}/task_cmd"


def robot_task_status_topic(robot_name: str) -> str:
    return f"/{robot_name}/task_status"


def robot_task_event_topic(robot_name: str) -> str:
    return f"/{robot_name}/task_event"


def robot_adapter_cmd_topic(robot_name: str) -> str:
    return f"/{robot_name}/adapter_cmd"


def robot_cmd_mqtt_topic(robot_name: str) -> str:
    return f"robot/{robot_name}/cmd"


def robot_state_mqtt_topic(robot_name: str) -> str:
    return f"robot/{robot_name}/state"


def robot_event_mqtt_topic(robot_name: str) -> str:
    return f"robot/{robot_name}/event"

