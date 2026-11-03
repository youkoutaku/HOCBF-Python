"""Closed-loop regression: actual avoidance, settled goal, and retained evidence."""
import json
from pathlib import Path
import tempfile
import unittest

import numpy as np

from hocbf.models import KINDS
from hocbf.single_obstacle import DemoModel, GOAL, OBSTACLES, RADII, run_model


class SingleObstacle(unittest.TestCase):
    def test_straight_reference_is_blocked_and_initial_centers_match(self):
        self.assertEqual(GOAL[1], 0.0)
        self.assertLess(abs(OBSTACLES[0, 1]), RADII[0])
        for kind in KINDS:
            model = DemoModel(kind, vmax=0.7)
            np.testing.assert_allclose(model.center(model.initial()), [0.0, 0.0])

    def test_all_four_models_avoid_and_settle_with_valid_records(self):
        with tempfile.TemporaryDirectory() as directory:
            for kind in KINDS:
                with self.subTest(model=kind):
                    folder = Path(directory) / kind
                    summary = run_model(kind, folder)
                    self.assertEqual(summary['outcome'], 'goal_complete')
                    self.assertGreater(summary['minimum_sampled_clearance'], 0.0)
                    self.assertGreaterEqual(summary['settled_dwell'], 0.5 - 1e-9)
                    self.assertGreater(summary['maximum_lateral_excursion'], 0.1)
                    with np.load(folder / 'trajectory.npz', allow_pickle=False) as data:
                        self.assertEqual(len(data['state']), len(data['control']) + 1)
                        self.assertEqual(data['dense_center'].shape, (len(data['control']), 21, 2))
                        actual = np.min(np.linalg.norm(data['dense_center'] - OBSTACLES[0], axis=-1) - RADII[0])
                        self.assertAlmostEqual(actual, summary['minimum_sampled_clearance'], places=12)
                        events = json.loads((folder / 'events.json').read_text())
                        self.assertEqual(len(events), len(data['control']))
                    with self.assertRaises(FileExistsError):
                        run_model(kind, folder)

    def test_direct_goal_limitation_remains_visible(self):
        with tempfile.TemporaryDirectory() as directory:
            summary = run_model('kinematic_unicycle', Path(directory) / 'direct',
                                steps=400, guidance='direct')
            self.assertEqual(summary['guidance'], 'direct_goal')
            self.assertEqual(summary['outcome'], 'timeout')
            self.assertGreater(summary['final_goal_error'], 2.0)


if __name__ == '__main__':
    unittest.main()
