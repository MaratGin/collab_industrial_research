#!/usr/bin/env python3
from real_case_impl.sim.task_executor import ConfigDrivenTaskExecutor, run_executor


class URSimTaskExecutorNode(ConfigDrivenTaskExecutor):
    def __init__(self) -> None:
        super().__init__("ur_sim_executor_node", "ur5e")


def main(args=None) -> None:
    run_executor(URSimTaskExecutorNode, "UR sim executor interrupted by user", args=args)


if __name__ == "__main__":
    main()
