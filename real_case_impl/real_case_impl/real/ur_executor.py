from pathlib import Path

from ament_index_python.packages import get_package_share_directory

from real_case_impl.sim.task_executor import ConfigDrivenTaskExecutor


class URRealTaskExecutorNode(ConfigDrivenTaskExecutor):
    def __init__(self) -> None:
        super().__init__("ur_real_executor_node", "ur5e")

    def _resolve_config_path(self) -> str:
        if self.task_config_path:
            return self.task_config_path
        share_dir = get_package_share_directory("real_case_impl")
        return str(Path(share_dir) / "config" / "real_robot_task_sequences.yaml")

