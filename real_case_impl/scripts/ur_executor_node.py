#!/usr/bin/env python3
from real_case_impl.sim.task_executor import ConfigDrivenTaskExecutor, run_executor


class URTaskExecutorNode(ConfigDrivenTaskExecutor):
    def __init__(self) -> None:
        super().__init__("ur_task_executor_node", "ur5e")


def main(args=None) -> None:
    run_executor(URTaskExecutorNode, "UR Task Executor interrupted by user", args=args)


if __name__ == "__main__":
    main()
