"""Smoke tests for the strategy module template.

Replace or extend these with behavior-focused tests as strategy code is implemented.
"""

from src.strategy import combat, loot, navigation, survival, targeting, upgrades


def test_strategy_modules_are_importable() -> None:
    modules = (combat, loot, navigation, survival, targeting, upgrades)

    assert all(module.__doc__ for module in modules)
