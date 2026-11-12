"""Ensure robustness means distinct physical conditions with shared control."""

from walking_benchmark import CASES


def test_fixed_conditions_cover_force_friction_and_mass():
    assert len(CASES) == 10
    assert len({c.name for c in CASES}) == 10
    assert {c.mass_ratio for c in CASES} == {0.95, 1.0, 1.05}
    assert {c.friction for c in CASES} == {0.4, 0.6}
    assert {c.push_axis for c in CASES if c.push_force} == {0, 1}
    assert {c.push_force for c in CASES} == {-10.0, 0.0, 10.0}
    assert len({(c.steps, c.step_length) for c in CASES}) == 1
