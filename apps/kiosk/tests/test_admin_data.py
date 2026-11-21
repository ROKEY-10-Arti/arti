import json
import sqlite3
import tempfile
import unittest
from datetime import datetime
from html.parser import HTMLParser
from pathlib import Path
from zoneinfo import ZoneInfo
from app import create_app
from tests.auth_helpers import login_admin


def kst(value):
    return int(datetime.fromisoformat(value).replace(tzinfo=ZoneInfo('Asia/Seoul')).timestamp() * 1000)


class AdminDataTest(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.config = {'TESTING':True, 'DATABASE':Path(self.temp.name)/'data.sqlite3',
                       'ROS_PREPARATION_REQUIRED':False, 'LOG_API_TOKEN':'test-log-token-for-ingestion'}
        self.app = create_app(self.config)
        self.client = login_admin(self.app.test_client())
        self.rows = [
            ('tb1',kst('2026-11-16T23:59:59'),'moving'),
            ('tb4',kst('2026-11-17T00:00:00'),'arrived_% sample'),
            ('tb1',kst('2026-11-17T23:59:59'),'visitor registered'),
            ('tb4',kst('2026-11-18T00:00:00'),'ready'),
        ]
        for robot,timestamp,message in self.rows:
            self.assertEqual(self.client.post('/api/logs',json={'robot_id':robot,'sent_at':timestamp,'message':message}).status_code,201)

    def test_login_table_forms_and_direct_row_edit_reach_public_kiosk_api(self):
        class Parser(HTMLParser):
            def __init__(self): super().__init__(); self.forms={}; self.controls={}
            def handle_starttag(self,tag,attributes):
                attrs=dict(attributes)
                if tag=='form' and 'id' in attrs:self.forms[attrs['id']]=attrs
                if tag in ('input','textarea') and 'form' in attrs:self.controls.setdefault(attrs['form'],set()).add(attrs['name'])
        html=self.client.get('/data').get_data(as_text=True)
        parser=Parser();parser.feed(html)
        for index in range(1,5):
            form_id=f'art-row-{index:03d}'
            self.assertIn(form_id,parser.forms)
            self.assertEqual(parser.controls[form_id],{'artwork_title','artist_name','location','theme','description','x','y'})
        for label in ('작품 DB · Art', 'artwork_title', 'artist_name'):
            self.assertIn(label, html)
        payload={'artwork_title':'새 작품 제목','artist_name':'새 작가',
                 'location':'room-7','theme':'새 테마','description':'수정한 설명','x':1.25,'y':-3}
        self.assertEqual(self.client.patch('/api/artworks/001',json=payload).status_code,200)
        public=self.app.test_client().get('/api/catalog').json['artworks'][0]
        for key,value in payload.items():self.assertEqual(public[key],value)
        self.assertEqual(create_app(self.config).test_client().get('/api/artworks/001').json,public)

    def test_log_filters_use_sender_time_kst_robot_and_message(self):
        data=self.client.get('/api/logs',query_string={'from':'2026-11-17','to':'2026-11-17'}).json
        self.assertEqual(data['total'],2)
        self.assertEqual([row['log_id'] for row in data['logs']],[3,2])
        data=self.client.get('/api/logs',query_string={'from':'2026-11-17','to':'2026-11-17','robot_id':'tb4','q':'arrived'}).json
        self.assertEqual([row['log_id'] for row in data['logs']],[2])
        page=self.client.get('/data',query_string={'q':'arrived'}).get_data(as_text=True)
        self.assertIn('arrived_% sample',page);self.assertNotIn('visitor registered',page)
        self.assertIn('value="arrived"',page)

    def test_search_is_literal_parameterized_and_read_only(self):
        for query,count in (('%',1),('_',1),('tb1',2),('arrived',1),("' OR 1=1 --",0)):
            with self.subTest(query=query):
                result=self.client.get('/api/logs',query_string={'q':query})
                self.assertEqual(result.status_code,200);self.assertEqual(result.json['total'],count)
        with sqlite3.connect(self.config['DATABASE']) as db:
            self.assertEqual(db.execute('SELECT robot_id,sent_at,message FROM Log ORDER BY log_id').fetchall(),self.rows)

    def test_pagination_and_exact_four_column_contract(self):
        all_rows=self.client.get('/api/logs').json['logs'];seen=[]
        for offset in range(4):
            data=self.client.get('/api/logs',query_string={'limit':1,'offset':offset}).json
            self.assertEqual(data['total'],4);self.assertEqual(data['has_more'],offset<3)
            self.assertEqual(set(data['logs'][0]),{'log_id','robot_id','sent_at','message'})
            seen.extend(data['logs'])
        self.assertEqual(seen,all_rows)
        self.assertEqual(self.client.get('/api/logs?offset=99').json['logs'],[])
        with sqlite3.connect(self.config['DATABASE']) as db:
            self.assertEqual([row[1] for row in db.execute('PRAGMA table_info(Log)')],['log_id','robot_id','sent_at','message'])

    def test_operator_log_explains_direction_status_and_failures_for_existing_rows(self):
        ros_rows = [
            ('robot4', '/robot4/arti/prepare_tour sending goal start=true'),
            ('robot4', '/robot4/arti/prepare_tour goal accepted=True'),
            ('robot4', '/robot4/arti/prepare_tour result status=6 success=False message=AMCL/map_server/Nav2 lifecycle not active'),
            ('robot1', '/robot1/arti/start_tour feedback state=EXPLAINING artwork_id=2'),
            ('robot1', '/robot1/arti/start_tour result status=5 success=True message=canceled'),
            ('kiosk', '/arti/description request artwork_id=1; response success=True description_chars=25'),
        ]
        with sqlite3.connect(self.config['DATABASE']) as db:
            db.executemany('INSERT INTO Log (robot_id,sent_at,message) VALUES (?,0,?)', ros_rows)
        logs = self.client.get('/api/logs?view=operator').json['logs']
        outgoing, accepted, aborted, explaining, canceled, service = [row['operator'] for row in reversed(logs[-6:])]
        self.assertEqual((outgoing['sender'], outgoing['target']), ('키오스크', '로봇 4'))
        self.assertEqual((accepted['sender'], accepted['target']), ('로봇 4', '키오스크'))
        self.assertEqual(accepted['namespace'], '/robot4')
        self.assertIn('주행 시스템이 활성화되지 않았습니다', aborted['description'])
        self.assertIn('2번 작품 설명 중', explaining['description'])
        self.assertIn('취소를 확인했습니다', canceled['description'])
        self.assertEqual(service['target'], '요청 로봇(미확인)')
        self.assertEqual(logs[0]['operator']['sender'], '미확인')
        self.assertEqual(logs[0]['operator']['payload'], logs[0]['message'])
        actions = self.client.get('/api/logs?view=operator&kind=action&limit=2').json
        self.assertEqual(actions['total'], 5)
        self.assertTrue(actions['has_more'])
        self.assertTrue(all(row['operator']['type'].startswith('ROS 액션') for row in actions['logs']))
        services = self.client.get('/api/logs?kind=service').json
        self.assertEqual([row['message'] for row in services['logs']], [ros_rows[-1][1]])
        self.assertEqual(self.client.get('/api/logs?kind=action&robot_id=robot4').json['total'], 3)
        page = self.client.get('/data').get_data(as_text=True)
        for label in ('발신자', '2번 작품 설명 중', '통신 경로 (리소스)', '현재 활동'):
            self.assertIn(label, page)

    def test_fleet_uses_odometry_only_while_fresh(self):
        fleet = self.app.extensions['robot_availability'].fleet
        now = 10
        fleet.clock = lambda: now
        fleet.receive('tb1', moving=True)
        fleet.receive('tb4', dock=True)
        snapshot = self.client.get('/api/fleet').json
        self.assertEqual(snapshot['tb1']['activity'], '이동 중')
        self.assertEqual(snapshot['tb4']['activity'], '도크에 정차 중')
        now = 14
        snapshot = self.client.get('/api/fleet').json
        self.assertEqual(snapshot['tb1']['activity'], '활동 상태 미수신')

    def test_invalid_filters_and_unauthorized_access_are_rejected(self):
        for query in ({'from':'2026-02-30'},{'from':'2026-1-1'},{'to':'2026-11-16','from':'2026-11-17'},
                      {'robot_id':'x'*65},{'q':'x'*201},{'limit':0},{'limit':501},{'offset':-1},{'offset':1000001},{'to':'9999-12-31'}):
            with self.subTest(query=query):
                self.assertEqual(self.client.get('/api/logs',query_string=query).status_code,400)
                self.assertEqual(self.client.get('/data',query_string=query).status_code,400)
        self.assertEqual(self.app.test_client().get('/api/logs').status_code,401)
        self.assertEqual(self.app.test_client().get('/data').status_code,302)

    def test_external_http_log_requires_token_and_preserves_other_databases(self):
        sender=self.app.test_client()
        payload={'robot_id':'tb1','sent_at':'2026-11-17T19:23:45+09:00','message':'키오스크 도착'}
        with sqlite3.connect(self.config['DATABASE']) as db:
            before={table:db.execute(f'SELECT * FROM {table} ORDER BY 1').fetchall() for table in ('Art','Pose','TourSession')}
        self.assertEqual(sender.post('/api/logs',json=payload).status_code,401)
        self.assertEqual(sender.post('/api/logs',json=payload,headers={'Authorization':'Bearer wrong'}).status_code,401)
        headers={'Authorization':'Bearer '+self.config['LOG_API_TOKEN']}
        result=sender.post('/api/logs',json=payload,headers=headers,environ_overrides={'REMOTE_ADDR':'192.168.107.101'})
        self.assertEqual(result.status_code,201)
        self.assertEqual(result.json,{'log_id':5,'robot_id':'tb1','sent_at':kst('2026-11-17T19:23:45'),'message':'키오스크 도착'})
        self.assertEqual(sender.get('/api/logs',headers=headers).status_code,401)
        self.assertEqual(sender.delete('/api/logs',json={'confirm':'RESET_LOGS','expected_count':5},headers=headers).status_code,401)
        with sqlite3.connect(self.config['DATABASE']) as db:
            for table,rows in before.items():self.assertEqual(db.execute(f'SELECT * FROM {table} ORDER BY 1').fetchall(),rows)
        restarted=login_admin(create_app(self.config).test_client())
        self.assertEqual(restarted.get('/api/logs?q=키오스크').json['logs'][0],result.json)

    def test_sender_fields_validate_and_keep_message_as_text(self):
        base={'robot_id':'tb1','sent_at':12345,'message':'hello'}
        for patch in ({'robot_id':''},{'robot_id':'bad/id'},{'sent_at':True},{'sent_at':-1},{'sent_at':1.2},
                      {'sent_at':'2026-11-17T19:00:00'},{'message':''},{'message':' '*4},{'message':'a'*10001},{'log_id':1}):
            self.assertEqual(self.client.post('/api/logs',json={**base,**patch}).status_code,400)
        self.assertEqual(self.client.post('/api/logs',json={'robot_id':'tb1','message':'missing time'}).status_code,400)
        value='<img src=x onerror=alert(1)>'
        self.assertEqual(self.client.post('/api/logs',json={**base,'message':value}).status_code,201)
        page=self.client.get('/data').get_data(as_text=True)
        self.assertNotIn(value,page);self.assertIn('&lt;img',page)
        self.assertEqual(self.client.get('/api/logs').json['total'],5)

    def test_log_reset_preserves_sessions_and_restarts_index_at_one(self):
        self.client.post('/api/sessions',json={'mode':'free','artworkIds':['art-01']})
        tables=('Art','Pose','TourSession','courses','catalog_migrations','admin_users','admin_sessions')
        with sqlite3.connect(self.config['DATABASE']) as db:before={table:db.execute(f'SELECT * FROM {table} ORDER BY 1').fetchall() for table in tables}
        response=self.client.delete('/api/logs',json={'confirm':'RESET_LOGS','expected_count':4})
        self.assertEqual(response.json,{'deleted':4,'total':0})
        with sqlite3.connect(self.config['DATABASE']) as db:
            for table,rows in before.items():self.assertEqual(db.execute(f'SELECT * FROM {table} ORDER BY 1').fetchall(),rows)
        restarted=login_admin(create_app(self.config).test_client())
        self.assertEqual(restarted.get('/api/logs').json['logs'],[])
        self.assertEqual(len(restarted.get('/api/sessions').json['sessions']),1)
        created=restarted.post('/api/logs',json={'robot_id':'tb1','sent_at':1000,'message':'new log'})
        self.assertEqual(created.json['log_id'],1)

    def test_log_reset_requires_csrf_and_matching_count(self):
        payload={'confirm':'RESET_LOGS','expected_count':4}
        self.assertEqual(self.app.test_client().delete('/api/logs',json=payload).status_code,401)
        token=self.client.environ_base.pop('HTTP_X_CSRF_TOKEN')
        self.assertEqual(self.client.delete('/api/logs',json=payload).status_code,403)
        self.client.environ_base['HTTP_X_CSRF_TOKEN']=token
        for body in ({},{'confirm':'wrong','expected_count':4},{'confirm':'RESET_LOGS','expected_count':True}):
            self.assertEqual(self.client.delete('/api/logs',json=body).status_code,400)
        self.client.post('/api/logs',json={'robot_id':'tb1','sent_at':123,'message':'new'})
        self.assertEqual(self.client.delete('/api/logs',json=payload).status_code,409)
        self.assertEqual(self.client.get('/api/logs').json['total'],5)
