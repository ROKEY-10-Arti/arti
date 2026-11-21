"""관리 대상: 작품 정보."""

import math

from flask import abort, jsonify
from database import ARTWORK_SELECT


ARTWORK_FIELDS = {"artist_name": 200, "theme": 200, "artwork_title": 200, "description": 10000,
                  "image_alt": 500, "location": 64}


def register_managed_data(app, get_db, json_body):
    def find_artwork(artwork_id):
        row = get_db().execute(ARTWORK_SELECT + " WHERE artwork_id = ?", (artwork_id,)).fetchone()
        if row is None:
            abort(404, description="등록되지 않은 작품 ID입니다.")
        return dict(row)

    @app.get("/api/artworks")
    def list_artworks():
        return jsonify(artworks=[dict(row) for row in get_db().execute(
            ARTWORK_SELECT + " ORDER BY artwork_id")])

    @app.get("/api/artworks/<int:artwork_number>")
    def get_artwork(artwork_number):
        return jsonify(find_artwork(f"{artwork_number:03d}"))

    @app.patch("/api/artworks/<int:artwork_number>")
    def update_artwork(artwork_number):
        artwork_id = f"{artwork_number:03d}"
        find_artwork(artwork_id)
        body = json_body(set(ARTWORK_FIELDS) | {'x', 'y'})
        if not body:
            abort(400, description="수정할 작품 정보를 하나 이상 보내 주세요.")
        for key, value in body.items():
            if key in {'x', 'y'}:
                if value is not None and (isinstance(value, bool) or not isinstance(value, (int, float))
                        or not -1e12 <= value <= 1e12 or not math.isfinite(value)):
                    abort(400, description='작품 좌표는 ±1e12 범위의 유한한 숫자 또는 null이어야 합니다.')
                continue
            if (not isinstance(value, str) or len(value) > ARTWORK_FIELDS[key]
                    or (key != 'image_alt' and not value.strip())):
                abort(400, description=f"{key}는 비어 있지 않은 {ARTWORK_FIELDS[key]}자 이내의 문자열이어야 합니다.")
        # 열 이름은 위의 고정된 허용 목록으로 제한하며, 값은 매개변수로 전달합니다.
        fields = ", ".join(f"{key} = ?" for key in body)
        db = get_db()
        with db:
            db.execute(f"UPDATE Art SET {fields} WHERE artwork_id = ?",
                       [value.strip() if isinstance(value, str) else value for value in body.values()] + [artwork_id])
        return jsonify(find_artwork(artwork_id))
