#!/usr/bin/env python3
from real_case_impl.common.adapter import KukaAdapterNode, run_node


def main(args=None) -> None:
    run_node(KukaAdapterNode, "KUKA Adapter interrupted by user", args=args)


if __name__ == "__main__":
    main()
