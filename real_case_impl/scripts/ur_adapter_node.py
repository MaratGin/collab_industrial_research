#!/usr/bin/env python3
from real_case_impl.common.adapter import URAdapterNode, run_node


def main(args=None) -> None:
    run_node(URAdapterNode, "UR Adapter interrupted by user", args=args)


if __name__ == "__main__":
    main()
