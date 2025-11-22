"""Tests pour le module de scoring."""

import unittest
import sys
sys.path.insert(0, '..')

from mam_python.scoring.score_tracker import ScoreTracker
from mam_python.scoring.score_predictor import ScorePredictor


class TestScoreTracker(unittest.TestCase):
    """Tests du tracker de score."""
    
    def setUp(self):
        self.tracker = ScoreTracker()
    
    def test_add_points(self):
        """Test l'ajout de points."""
        self.tracker.add_points(100, "crate_collected")
        self.assertEqual(self.tracker.get_total_score(), 100)
    
    def test_multiplier(self):
        """Test le multiplicateur."""
        self.tracker.set_multiplier(2.0)
        self.tracker.add_points(100, "crate_collected")
        self.assertEqual(self.tracker.get_total_score(), 200)


class TestScorePredictor(unittest.TestCase):
    """Tests du prédicteur de score."""
    
    def setUp(self):
        self.predictor = ScorePredictor()
    
    def test_prediction(self):
        """Test la prédiction."""
        conservative, optimistic = self.predictor.predict_final_score(5, 30, 90)
        self.assertEqual(conservative, 500)
        self.assertGreaterEqual(optimistic, conservative)


if __name__ == '__main__':
    unittest.main()
