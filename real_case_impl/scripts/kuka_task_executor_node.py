#!/usr/bin/env python3
from real_case_impl.sim.task_executor import ConfigDrivenTaskExecutor, run_executor


class KukaTaskExecutorNode(ConfigDrivenTaskExecutor):
    def __init__(self) -> None:
        super().__init__("kuka_task_executor_node", "kuka")


def main(args=None) -> None:
    run_executor(KukaTaskExecutorNode, "KUKA Task Executor interrupted by user", args=args)


if __name__ == "__main__":
    main()
