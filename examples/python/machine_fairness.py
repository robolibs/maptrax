"""Side-by-side comparison of all division modes. Terminal output only —
use machine_optimized.py if you want the winner in Rerun.
"""

import maptrax

from _common import BIG_IRREGULAR_FIELD, DATUM, summarise_machines


SCENARIOS = [
    ("Stripe{stride:1} + ByCount", {"pattern": "stripe", "balance": "count", "stride": 1}),
    ("Stripe{stride:2} + ByCount", {"pattern": "stripe", "balance": "count", "stride": 2}),
    ("Block + ByCount", {"pattern": "block", "balance": "count"}),
    ("Block + ByLength (smart blocks)", {"pattern": "block", "balance": "length"}),
    ("BandedStripe{2} + ByLength", {"pattern": "banded_stripe", "bands": 2, "balance": "length"}),
    ("Optimized{Makespan} + ByLength", {"pattern": "optimized_makespan", "balance": "length"}),
]


def main():
    planner = maptrax.Maptrax()
    planner.set_field(BIG_IRREGULAR_FIELD, DATUM)
    planner.generate_field(8.0, 20.0, 3)

    print("=== Fairness comparison: 3 machines, Reeds-Shepp turns ===\n")
    for label, kwargs in SCENARIOS:
        plan = planner.plan_machines(
            part_index=0,
            machines=3,
            routing_strategy="greedy_nearest",
            local_improvement_passes=1,
            turn_model="reeds_shepp",
            min_turning_radius=3.0,
            swath_width=8.0,
            machine_length=6.0,
            machine_width=3.0,
            **kwargs,
        )
        print(f"--- {label} ---")
        if "pattern_used" in plan:
            print(f"    resolved: {plan['pattern_used']}")
        makespan, total = summarise_machines(plan)
        print(f"    makespan: {makespan:.1f}s   total_work: {total:.1f} m\n")


if __name__ == "__main__":
    main()
