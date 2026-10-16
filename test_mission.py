import json, unittest
from pathlib import Path
from mission import Environment, Agent
ROOT=Path(__file__).resolve().parents[1]
class MissionTests(unittest.TestCase):
 def setUp(self):
  self.cfg=json.loads((ROOT/"configs/sample_mission.json").read_text())
  self.env=Environment(self.cfg,seed=7);self.agent=Agent(self.env)
 def test_start_is_valid(self):
  self.assertTrue(self.env.inside(self.env.start));self.assertNotIn(self.env.start,self.env.obstacles)
 def test_belief_normalized(self):
  self.assertAlmostEqual(sum(self.agent.belief.values()),1.0,places=6)
 def test_hidden_location_not_in_normal_observation(self):
  obs=self.env.observe()
  self.assertNotIn("eve",json.dumps(obs).lower())
 def test_blocked_move_observation(self):
  self.env.pos=(2,0);obs=self.env.step({"type":"move","delta":[1,0]})
  self.assertEqual(obs["kind"],"blocked_move")
 def test_run_has_trace_and_report(self):
  report=self.agent.run(max_steps=100)
  self.assertTrue(report["trace"]);self.assertIn("recommended_next_step",report)
if __name__=="__main__":unittest.main()
