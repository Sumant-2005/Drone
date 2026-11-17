import importlib
import json
from fractions import Fraction
from pathlib import Path

from dronesim import params


def check(label, expected, got):
    passed = got == expected
    print(f"{label}: {expected} | {got} | {'PASS' if passed else 'FAIL'}")
    assert passed


def test_skeleton():
    package = Path(params.__file__).parent
    root = package.parents[1]
    for name in ("body", "mixer", "motors", "control", "mission", "sim"):
        module = importlib.import_module(f"dronesim.{name}")
        check(f"import {name}", f"{name}.py", Path(module.__file__).relative_to(package).as_posix())
    values = {k: v for k, v in vars(params).items() if not k.startswith("_")}
    check("S0 parameters", {"tilt_compensation": 1}, values)

    # The brief specifies 1.25 ms. Construct an exact reference grid in seconds.
    step = Fraction("1.25") / 1000
    steps = int(1 / step)
    check("one second on the fixed-step grid", Fraction(1), steps * step)
    check("fixed step (s)", 0.00125, float(step))
    checks = {
        "fixed_step_s": {"value": float(step), "atol": 0},
        "top_level_blocks": {"value": 0, "atol": 0},
        "solver": {"value": "ode4"},
        "solver_type": {"value": "Fixed-step"},
    }
    for name, data in (("params", values), ("stage0", checks)):
        path = root / "expected" / f"{name}.json"
        path.write_text(json.dumps(data, indent=2) + "\n", encoding="utf-8")
