import math
import sys
import unittest
from pathlib import Path
from types import SimpleNamespace

import numpy as np
from scipy.ndimage import distance_transform_edt

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from navigation import GridMap, cell_of, wrap
from courier_node import Courier


class NavigationTests(unittest.TestCase):
    def test_stuck_detection_runs_before_any_planner_early_return(self):
        actions = []
        robot = SimpleNamespace(goal=np.array([1., 0.]), pose=np.zeros(3),
                                progress_pose=np.zeros(2), progress_time=0., state='MOVE',
                                now=lambda: 10., drive=lambda **kw: actions.append(kw),
                                transition=actions.append)
        self.assertFalse(Courier.navigate(robot))
        self.assertEqual(robot.recovery_state, 'MOVE')
        self.assertEqual(actions, [{'emergency': True}, 'RECOVER'])

    def test_emergency_stop_clears_both_velocity_commands(self):
        published = []
        robot = SimpleNamespace(command=np.array([.6, 1.2]),
                                cmd_pub=SimpleNamespace(publish=published.append))
        Courier.drive(robot, emergency=True)
        self.assertEqual(published[-1].linear.x, 0.)
        self.assertEqual(published[-1].angular.z, 0.)
        Courier.drive(robot, .65, 1.4)
        self.assertAlmostEqual(published[-1].linear.x, .045)
        self.assertAlmostEqual(published[-1].angular.z, .22)

    def test_shortest_return_route_and_unreachable_goal(self):
        grid = GridMap()
        grid.graph = {(0, 0): {(1, 0)}, (1, 0): {(0, 0), (1, 1)}, (1, 1): {(1, 0)}}
        self.assertEqual(grid.route((1, 1), {(0, 0)}), [(1, 1), (1, 0), (0, 0)])
        self.assertEqual(grid.route((0, 0), {(2, 2)}), [])
        grid.block_edge((1, 0), (1, 1))
        self.assertFalse(grid.route((0, 0), {(1, 1)}))
        self.assertAlmostEqual(wrap(3*math.pi), math.pi)
        self.assertEqual(cell_of(-4.1, 2.05), (-2, 1))

    def test_pallet_detour_with_body_clearance(self):
        grid = GridMap()
        x, y = grid.pixel(1, 0)
        grid.evidence[y-5:y+6, x-5:x+6] = 4
        grid.clearance = distance_transform_edt(grid.evidence < .6)*grid.resolution
        path = grid.plan((0, 0), (2, 0))
        self.assertTrue(path)
        self.assertGreater(max(abs(y) for _, y in path), .45)
        self.assertTrue(all(grid.line_clear(a, b) for a, b in zip(path, path[1:])))
        self.assertFalse(grid.line_clear((0, 0), (2, 0)))
        goal = grid.cell_goal((0, 0))
        self.assertLess(np.linalg.norm(goal), .01)
        goal = grid.cell_goal((1, 0))
        px, py = grid.pixel(*goal)
        self.assertGreater(grid.clearance[py, px], .27)

    def test_closed_wall_is_not_a_passage(self):
        grid = GridMap()
        grid.evidence.fill(-3)
        x, _ = grid.pixel(1, 0)
        grid.evidence[:, x] = 4
        grid.clearance = distance_transform_edt(grid.evidence < .6)*grid.resolution
        grid.update_graph()
        self.assertNotIn((1, 0), grid.graph[(0, 0)])
        self.assertFalse(grid.plan((0, 0), (2, 0)))

    def test_goal_does_not_jump_when_a_pallet_observation_changes(self):
        grid = GridMap()
        x, y = grid.pixel(2, 0)
        grid.evidence[y-4:y+5, x-4:x+5] = 4
        grid.clearance = distance_transform_edt(grid.evidence < .6)*grid.resolution
        goal = grid.cell_goal((1, 0))
        self.assertGreater(np.linalg.norm(goal-(2, 0)), .4)
        grid.evidence[y-7:y+8, x-7:x+8] = 4
        grid.clearance = distance_transform_edt(grid.evidence < .6)*grid.resolution
        replacement = grid.cell_goal((1, 0), goal)
        self.assertGreater(np.dot(replacement-(2, 0), goal-(2, 0)), 0)
        grid.clearance.fill(8.)
        np.testing.assert_equal(grid.cell_goal((1, 0), goal), goal)

    def test_no_return_does_not_create_a_wall_and_nan_is_ignored(self):
        grid = GridMap()
        grid.integrate((0, 0, 0), np.array([0., math.pi/2]), np.array([math.inf, math.nan]), 8.)
        self.assertFalse(np.any(grid.evidence > 0))
        px, py = grid.pixel(1, 0)
        self.assertLess(grid.evidence[py, px], 0)


if __name__ == '__main__':
    unittest.main()
