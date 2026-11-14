"""Legacy Empty events must never advance either floor."""
from types import SimpleNamespace
from multifloor_exploration.coordinator import MultifloorCoordinator, State

def test_empty_does_not_authorize_legacy_coordinator():
    for state in (State.FLOOR_1, State.FLOOR_2):
        warnings = []
        calls = []
        fixture = SimpleNamespace(state=state,
            get_logger=lambda: SimpleNamespace(warning=warnings.append),
            call_frontier=lambda *args: calls.append(args),
            publish_state=lambda *args: calls.append(args))
        MultifloorCoordinator.exploration_complete(fixture, None)
        assert fixture.state is state
        assert not calls
        assert len(warnings) == 1
        assert "full_multifloor_demo.launch.py" in warnings[0]
