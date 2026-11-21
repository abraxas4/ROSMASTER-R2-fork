#!/usr/bin/env python3
"""
Unit test for direction alternation logic in path reverse recovery.
"""
import unittest


class DirectionAlternator:
    """Helper class to manage steering direction alternation"""
    def __init__(self):
        self.last_direction = 0
        
    def get_next_direction(self):
        """Get next steering direction, alternating left(1) and right(-1)"""
        if self.last_direction == 0:
            self.last_direction = 1
        else:
            self.last_direction = -self.last_direction
        return self.last_direction
        
    def reset(self):
        """Reset to initial state"""
        self.last_direction = 0


class TestDirectionAlternation(unittest.TestCase):
    
    def test_initial_direction(self):
        """First call should return 1 (left)"""
        alternator = DirectionAlternator()
        direction = alternator.get_next_direction()
        self.assertEqual(direction, 1, "First direction should be 1 (left)")
        
    def test_alternation_sequence(self):
        """Should alternate: 1, -1, 1, -1, ..."""
        alternator = DirectionAlternator()
        expected = [1, -1, 1, -1, 1, -1]
        actual = [alternator.get_next_direction() for _ in range(6)]
        self.assertEqual(actual, expected, "Direction should alternate left/right")
        
    def test_reset(self):
        """Reset should return to initial state"""
        alternator = DirectionAlternator()
        alternator.get_next_direction()
        alternator.get_next_direction()
        alternator.reset()
        direction = alternator.get_next_direction()
        self.assertEqual(direction, 1, "After reset, first direction should be 1")
        
    def test_string_representation(self):
        """Test that direction values map correctly to left/right"""
        alternator = DirectionAlternator()
        directions = []
        for _ in range(4):
            d = alternator.get_next_direction()
            if d > 0:
                directions.append("LEFT")
            else:
                directions.append("RIGHT")
        expected = ["LEFT", "RIGHT", "LEFT", "RIGHT"]
        self.assertEqual(directions, expected, "Direction should alternate LEFT/RIGHT")


if __name__ == '__main__':
    unittest.main()
