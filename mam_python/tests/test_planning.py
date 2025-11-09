"""Tests pour le module de planification."""

import unittest
import sys
sys.path.insert(0, '..')

from planning.path_planner import PathPlanner
from planning.collision_checker import CollisionChecker


class TestPathPlanner(unittest.TestCase):
    """Tests du planificateur de chemin."""
    
    def setUp(self):
        self.planner = PathPlanner()
    
    def test_line_planning(self):
        """Test la planification linéaire."""
        waypoints = self.planner.plan_line(0, 0, 1, 1, 5)
        self.assertEqual(len(waypoints), 6)
        self.assertAlmostEqual(waypoints[0].x, 0)
        self.assertAlmostEqual(waypoints[-1].x, 1)
    
    def test_arc_planning(self):
        """Test la planification en arc."""
        waypoints = self.planner.plan_arc(0, 0, 1, 0, 3.14, 10)
        self.assertEqual(len(waypoints), 11)


class TestCollisionChecker(unittest.TestCase):
    """Tests du vérificateur de collision."""
    
    def setUp(self):
        self.checker = CollisionChecker(0.2)
        self.checker.add_obstacle(1, 1, 0.3)
    
    def test_no_collision(self):
        """Test sans collision."""
        self.assertFalse(self.checker.check_collision(0, 0))
    
    def test_collision(self):
        """Test avec collision."""
        self.assertTrue(self.checker.check_collision(1, 1))


if __name__ == '__main__':
    unittest.main()
