#!/usr/bin/env python3
"""check_hilbench_layers.py — keeps the core of benchmarks/hilbench/ free of kind names and upward imports.

    python3 scripts/check_hilbench_layers.py        (exit 0 = clean, exit 1 = problems printed)

Rule 1 (names): no .py file under hilbench/ outside kinds/ and factory.py may contain the word
`controller` or `slam` (any case, code, comments and strings). The one allowed identifier is
`controller_cores` (the approved setting name). The tests/ folder is not part of the core.

Rule 2 (layers, CONTRACT.md section 0): a module imports only modules of a LOWER layer.
    L0 util, bench_geometry   L1 config   L2 supervisor net pi_state container gate model_check overhead
    L3 trial ui summary       L4 session  L5 kinds/*             L6 factory
A kind never imports another kind, except kinds/slam.py -> kinds/slam_probe.py.
One import edge goes up on purpose: a kind may import `register` from factory (the decorator lives there).
"""
import ast
import re
import sys
from pathlib import Path

HILBENCH = Path(__file__).resolve().parents[1] / "benchmarks" / "hilbench"
PACKAGE = "hilbench"

LAYERS = {
    "util": 0, "bench_geometry": 0,
    "config": 1,
    "supervisor": 2, "net": 2, "pi_state": 2, "container": 2, "gate": 2, "model_check": 2, "overhead": 2,
    "trial": 3, "ui": 3, "summary": 3,
    "session": 4,
    "kinds": 5,
    "factory": 6,
}
WORDS = re.compile(r"controller|slam", re.IGNORECASE)
ALLOWED_IDENTIFIER = "controller_cores"
SKIP_DIRS = {"__pycache__", "tests"}


def python_files(root):
    for p in sorted(Path(root).rglob("*.py")):
        if not (set(p.relative_to(root).parts[:-1]) & SKIP_DIRS):
            yield p


def check_names(root=None):
    """list[str]: one problem per line that holds `controller` or `slam` outside kinds/ and factory.py."""
    root = Path(root or HILBENCH)
    problems = []
    for p in python_files(root):
        rel = p.relative_to(root)
        if rel.parts[0] == "kinds" or rel.as_posix() == "factory.py":
            continue
        for n, line in enumerate(p.read_text().splitlines(), 1):
            for m in WORDS.finditer(line.replace(ALLOWED_IDENTIFIER, "")):
                problems.append(f"{rel}:{n}: kind name '{m.group(0)}' in the core: {line.strip()[:80]}")
                break
    return problems


def _module_parts(path, root):
    """('kinds', 'slam') for kinds/slam.py; ('config',) for config.py; ('kinds',) for kinds/__init__.py."""
    parts = list(path.relative_to(root).with_suffix("").parts)
    return tuple(parts[:-1]) if parts[-1] == "__init__" else tuple(parts)


def _targets(node, pkg, root):
    """The hilbench modules (tuples of names) that one import statement reaches, with the imported names."""
    out = []
    if isinstance(node, ast.Import):
        for a in node.names:
            parts = tuple(a.name.split("."))
            if parts[0] == PACKAGE:
                out.append((parts[1:], ()))
            elif parts[0] in LAYERS and len(parts) == 1:        # top-level bench_geometry
                out.append((parts, ()))
        return out
    if node.level:                                              # relative import
        base = pkg[:len(pkg) - (node.level - 1)] if node.level > 1 else pkg
        mod = base + tuple(node.module.split(".")) if node.module else base
    else:
        parts = tuple((node.module or "").split("."))
        if parts[0] == PACKAGE:
            mod = parts[1:]
        elif parts[0] in LAYERS and len(parts) == 1:
            mod = parts
        else:
            return out
    names = tuple(a.name for a in node.names)
    for a in names:                                             # `from . import x` may name a submodule
        sub = mod + (a,)
        if mod and (root.joinpath(*sub).with_suffix(".py").exists() or root.joinpath(*sub).is_dir()):
            out.append((sub, (a,)))
        elif not mod and root.joinpath(a + ".py").exists():
            out.append(((a,), ()))
    if not out or mod:
        out.append((mod, names))
    return out


def check_imports(root=None):
    """list[str]: one problem per import that goes up a layer, sideways, or from one kind to another."""
    root = Path(root or HILBENCH)
    problems = []
    for p in python_files(root):
        me = _module_parts(p, root)
        if not me:
            continue
        rel = p.relative_to(root)
        pkg = me if p.name == "__init__.py" else me[:-1]
        my_layer = LAYERS.get(me[0])
        if my_layer is None:
            problems.append(f"{rel}:1: module '{me[0]}' has no layer; add it to LAYERS and to CONTRACT.md")
            continue
        tree = ast.parse(p.read_text(), filename=str(p))
        for node in ast.walk(tree):
            if not isinstance(node, (ast.Import, ast.ImportFrom)):
                continue
            for target, names in _targets(node, pkg, root):
                if not target or target == me:
                    continue
                top = target[0]
                t_layer = LAYERS.get(top)
                if t_layer is None:
                    problems.append(f"{rel}:{node.lineno}: imports unknown hilbench module '{'.'.join(target)}'")
                    continue
                msg = _judge(me, my_layer, target, t_layer, names)
                if msg:
                    problems.append(f"{rel}:{node.lineno}: {msg}")
    return sorted(set(problems))


def _judge(me, my_layer, target, t_layer, names):
    """None when the import is allowed, else the reason."""
    name = ".".join(target)
    if me[0] == "kinds":
        if target[0] == "kinds":
            ok_kinds = {"base", me[-1]} if len(me) > 1 else set()
            if me[-2:] == ("kinds", "slam"):
                ok_kinds.add("slam_probe")
            if len(target) == 1 or target[1] in ok_kinds:
                return None
            return f"kind '{me[-1]}' imports another kind '{name}'"
        if target[0] == "factory" and set(names) <= {"register"} and names:
            return None
        if t_layer >= my_layer:
            return f"kind imports '{name}' (layer L{t_layer}, not below L{my_layer})"
        return None
    if t_layer >= my_layer:
        return f"'{me[0]}' (L{my_layer}) imports '{name}' (L{t_layer}): only lower layers are allowed"
    return None


def main():
    problems = check_names() + check_imports()
    for p in problems:
        print(p)
    if problems:
        print(f"{len(problems)} problem(s)")
        return 1
    print("hilbench layers: clean")
    return 0


if __name__ == "__main__":
    sys.exit(main())
