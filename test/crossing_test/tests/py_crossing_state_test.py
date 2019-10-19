import unittest
from mamcts import CrossingState, CrossingStateEpisodeRunner
from test.crossing_test.pyviewer import PyViewer
import time


class PickleTests(unittest.TestCase):
    # def test_draw_state(self):
    #     viewer = PyViewer()
    #     state = CrossingState({})
    #     state.draw(viewer)
    #     viewer.show(block=True)

    def test_episode_runner(self):
        viewer = PyViewer()
        runner = CrossingStateEpisodeRunner(
            30,
            viewer)
        for _ in range(0, 30):
            runner.step()
            viewer.show()


if __name__ == '__main__':
    unittest.main()
