import json
import sqlite3
import tempfile
import unittest

from tests.auth_helpers import login_admin
from pathlib import Path

from app import ROOT, create_app


class CollectionTest(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.config = {'TESTING':True, 'DATABASE':Path(self.temp.name)/'collection.sqlite3'}
        self.catalog = json.loads((ROOT/'data/catalog.json').read_text())

    def test_supplied_collection_and_images_are_served_from_database(self):
        client = login_admin(create_app(self.config).test_client())
        artworks = client.get('/api/artworks').json['artworks']
        metadata = [{key: art[key] for key in seed if key not in {'x', 'y'}}
                    for art, seed in zip(artworks, self.catalog['artworks'], strict=True)]
        self.assertEqual(metadata, [{key: value for key, value in art.items() if key not in {'x', 'y'}}
                                    for art in self.catalog['artworks']])
        self.assertEqual(client.get('/api/catalog').json['artworks'], artworks)
        for art in artworks:
            response = client.get(art['image_path'])
            self.assertEqual(response.status_code, 200)
            self.assertEqual(response.mimetype, 'image/svg+xml' if art['image_path'].endswith('.svg') else 'image/png')
            self.assertEqual(response.data, (ROOT/art['image_path'].lstrip('/')).read_bytes())
            response.close()
        page = client.get('/data').get_data(as_text=True)
        for art in artworks:
            for key in ('artwork_title','artist_name','description','image_path'):
                self.assertIn(art[key], page)

    def test_existing_placeholder_database_upgrades_once_without_losing_sessions(self):
        with sqlite3.connect(self.config['DATABASE']) as db:
            db.execute('CREATE TABLE artworks (artwork_index INTEGER PRIMARY KEY, artist_name TEXT NOT NULL, theme TEXT NOT NULL, title TEXT NOT NULL, description TEXT NOT NULL)')
            db.execute("INSERT INTO artworks VALUES (1,'임시 작가','자연','임시 작품','임시 설명')")
        client = login_admin(create_app(self.config).test_client())
        self.assertEqual(client.get('/api/artworks/001').json['artwork_title'], self.catalog['artworks'][0]['artwork_title'])
        session = client.post('/api/sessions', json={'mode':'free','artworkIds':['art-01']}).json
        change = client.patch('/api/artworks/001', json={'artwork_title':'관리자가 수정한 제목'})
        self.assertEqual(change.status_code, 200)
        restarted = create_app(self.config).test_client()
        self.assertEqual(restarted.get('/api/artworks/001').json, change.json)
        self.assertEqual(restarted.get('/api/sessions/'+session['id']).json, session)

    def test_metadata_validation_and_persistent_edit(self):
        client = login_admin(create_app(self.config).test_client())
        original = client.get('/api/artworks/002').json
        for body in ({'year':True},{'year':'2026'},{'year':2026.5},{'year':0},{'year':10000},
                     {'curator_notes':[]},{'artwork_title':'a'*201},{'image_path':'https://example.com/image.png'},
                     {'artwork_code':'ART-999'},{'artwork_title':''},{'artist_name':' '}):
            with self.subTest(body=body):
                self.assertEqual(client.patch('/api/artworks/002',json=body).status_code,400)
                self.assertEqual(client.get('/api/artworks/002').json,original)
        body = {'artwork_title':'New Title','artist_name':'New Artist','description':'핵심 해설','image_alt':'그림 설명'}
        updated = client.patch('/api/artworks/002', json=body)
        self.assertEqual(updated.status_code,200)
        self.assertEqual(updated.json,{**original,**body})
        self.assertEqual(create_app(self.config).test_client().get('/api/artworks/002').json,updated.json)
        self.assertEqual(client.patch('/api/artworks/002',json={'image_alt':''}).status_code,200)


if __name__ == '__main__':
    unittest.main()
