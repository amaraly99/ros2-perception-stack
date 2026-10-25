#!/usr/bin/env python3
"""hilbench_run.py — run any HIL benchmark kind through the shared harness.

    benchmarks/hilbench_run.py controller [options]     # see: hilbench_run.py controller --help
    benchmarks/hilbench_run.py <kind> --dry-run

The kinds are the plug-ins in benchmarks/hilbench/kinds/. Use ~/.venvs/hilrun/bin/python (has rich + PyYAML).
"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))


def main(argv):
    from hilbench import factory
    names = factory.kinds_available()
    if not argv or argv[0] in ("-h", "--help") or argv[0] not in names:
        print(f"usage: hilbench_run.py <kind> [options]   kinds: {', '.join(names) or 'none'}\n"
              f"       hilbench_run.py <kind> --help", file=sys.stderr)
        return 0 if argv and argv[0] in ("-h", "--help") else 2
    return factory.main(argv[0], argv[1:])


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
