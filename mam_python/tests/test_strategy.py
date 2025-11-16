"""Tests pour le module de stratégie."""

import unittest
import sys
sys.path.insert(0, '..')

from mam_python.decision.strategy_engine import StrategyEngine, StrategyMode


class TestStrategyEngine(unittest.TestCase):
    """Tests du moteur de stratégie."""
    
    def setUp(self):
        self.engine = StrategyEngine()
    
    def test_mode_change(self):
        """Test le changement de mode."""
        self.engine.set_mode(StrategyMode.AGGRESSIVE)
        self.assertEqual(self.engine.mode, StrategyMode.AGGRESSIVE)
    
    def test_return_home_decision(self):
        """Test la décision de retour à la base."""
        self.engine.update_time(85, 90)
        decision = self.engine.decide_next_action((0, 0), [], 100)
        self.assertEqual(decision.action, "return_home")


if __name__ == '__main__':
    unittest.main()
