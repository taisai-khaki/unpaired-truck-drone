from src.data_models import SolutionData
from src.experiments.run_matched_architecture_diagnostic import _cross_truck_recoveries


def test_cross_truck_recovery_counter_matches_launch_and_retrieval_trucks() -> None:
    cross_truck = SolutionData(
        instance_name="cross_truck",
        status="feasible",
        objective=0.0,
        components={},
        drone_routes={1: [0, 2, 1, 3, 0]},
        u_drone={(1, 1): 1},
        z1={(2, 1, 1): 1},
        z2={(3, 1, 2): 1},
    )
    same_truck = SolutionData(
        instance_name="same_truck",
        status="feasible",
        objective=0.0,
        components={},
        drone_routes={1: [0, 2, 1, 3, 0]},
        u_drone={(1, 1): 1},
        z1={(2, 1, 1): 1},
        z2={(3, 1, 1): 1},
    )

    assert _cross_truck_recoveries(cross_truck) == 1
    assert _cross_truck_recoveries(same_truck) == 0
