#!/usr/bin/env python3
from real_case_impl.real.ur_executor import URRealTaskExecutorNode
from real_case_impl.sim.task_executor import run_executor


def main(args=None) -> None:
    run_executor(URRealTaskExecutorNode, "UR real executor interrupted by user", args=args)


if __name__ == "__main__":
    main()
