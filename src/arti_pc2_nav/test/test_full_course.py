"""TourMachine 단위 테스트 (ROS 없이 실행).

실행: source /opt/ros/jazzy/setup.bash && python3 -m pytest -q test/
반원 판정, 출구 반원, 도킹 확인, 출구/도킹 접근점 흐름, 복구(retry) 등을 검사합니다.
"""
import unittest
from arti_pc2_nav.course_engine import TourMachine
from arti_pc2_nav.guided_tour_plan import load_course_config,parse_course_request,Pose2D
from arti_pc2_nav.zone_geometry import exit_semicircle
from pathlib import Path
P=Path(__file__).resolve().parents[1]/'config/a_course.yaml'
class Tests(unittest.TestCase):
 def setUp(self):self.m=TourMachine(load_course_config(P));self.m.start(parse_course_request('A'),0,Pose2D(0,0))
 def test_half_plane(self):
  z=self.m.plan.artworks[0].zone
  self.assertFalse(z.contains((z.cx-.1,z.cy)));self.assertTrue(z.contains((z.cx+.1,z.cy)))
 def test_exit(self):
  z=exit_semicircle();self.assertAlmostEqual(z.cx,.157);self.assertAlmostEqual(z.cy,2.115);self.assertTrue(z.contains((.1,2.115)))
 def test_dock_needs_confirmation(self):
  self.m.state='DOCKING';events=self.m.docking_action_result(True,1);self.assertEqual(self.m.state,'WAIT_DOCK_CONFIRM');self.assertNotIn('COMPLETED',str(events));self.assertIn('COMPLETED',str(self.m.dock_confirmed(2)))
 def test_dock_fail(self):
  self.m.state='DOCKING';self.m.docking_action_result(False,1);self.assertEqual(self.m.state,'ERROR')
 def test_no_complete_after_art4(self):
  m=self.m;m.index=3;m.state='EXPLAINING';m.last_pose=m.plan.artworks[3].goal;m.last_pose_at=5
  m.explanation_result('4',True,5);self.assertEqual(m.state,'NAV_EXIT')
 def test_stop_requires_odom(self):
  m=self.m;m.state='VERIFY_STOP';p=m.plan.kiosk_goal
  m.tick(1,p,0,None,99);m.tick(2,p,0,None,99);self.assertEqual(m.state,'VERIFY_STOP')
 def test_full_mission_requires_every_artwork_and_physical_dock(self):
  m=self.m;t=1.;zones=[m.plan.kiosk_goal]+[a.goal for a in m.plan.artworks]
  for index,pose in enumerate(zones):
   m.tick(t,pose,0,0,0)
   self.assertEqual(m.state,'CANCELING')
   m.nav_result('CANCELED',t)
   m.tick(t+.1,pose,0,0,0);m.tick(t+1,pose,0,0,0)
   if index:
    self.assertEqual(m.state,'EXPLAINING');m.explanation_result(str(index),True,t+1)
   t+=2
  self.assertEqual(m.completed,['1','2','3','4']);self.assertEqual(m.state,'NAV_EXIT')
  ex=m.plan.exit_goal
  m.tick(t,ex,0,0,0);self.assertEqual(m.state,'CANCELING_EXIT');ev=m.nav_result('CANCELED',t);self.assertEqual(m.state,'VERIFY_EXIT_STOP')
  ev=m.tick(t+.1,ex,0,0,0)+m.tick(t+1,ex,0,0,0);self.assertEqual(m.state,'FAREWELL')
  self.assertIn('안녕히가세요',str(ev))
  ev=m.tick(t+3.5,ex,0,0,0);self.assertEqual(m.state,'NAV_HOME')
  self.assertEqual(m.home_goal,m.plan.dock_approach);self.assertIn('dock_approach',str(ev))
  t+=4
  m.tick(t,m.home_goal,0,0,0);self.assertEqual(m.state,'CANCELING_HOME');m.nav_result('CANCELED',t)
  m.tick(t+.1,m.home_goal,0,0,0);m.tick(t+1,m.home_goal,0,0,0)
  self.assertEqual(m.state,'DOCKING');m.docking_action_result(True,t+2)
  self.assertEqual(m.state,'WAIT_DOCK_CONFIRM');m.dock_confirmed(t+3)
  self.assertEqual(m.state,'DONE')
 def test_exit_far_requires_recovery(self):
  m=self.m;m.state='NAV_EXIT';m.exit_pending=True;m.last_pose=Pose2D(0.185,1.5);m.last_pose_at=1
  m.nav_result('SUCCEEDED',1);self.assertEqual(m.state,'WAIT_RECOVERY')
  self.assertIn("('exit'",str(m.retry(2)));self.assertEqual(m.state,'NAV_EXIT')
 def test_fixed_dock_approach_overrides_undock_pose(self):
  self.m.set_post_undock_pose(Pose2D(-0.111,-0.098));self.assertEqual(self.m.home_goal,Pose2D(-0.35,-0.0957,0.0))
if __name__=='__main__':unittest.main()
