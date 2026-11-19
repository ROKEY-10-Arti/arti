import assert from 'node:assert/strict';
import { courseSelection, selectedArtworks, sortedArtworks } from '../static/js/course-selection.js';

const artworks = [4, 2, 1, 3].map(index => ({
  artwork_id: `00${index}`,
  artwork_title: `artwork${index}`, artist_name: `artist${index}`,
  location: `00${index}`, theme: `theme${index}`, description: `description${index}`,
}));
assert.deepEqual(sortedArtworks(artworks).map(art => art.artwork_id), ['001', '002', '003', '004']);
assert.deepEqual(artworks.map(art => art.artwork_id), ['004', '002', '001', '003']);
// 체크한 순서나 중복과 관계없이 기존 작품을 숫자 인덱스 순서대로 한 번씩 표시합니다.
assert.deepEqual(selectedArtworks(artworks, ['004', 'missing', '001', '004']).map(art => art.artwork_id), ['001', '004']);
for (let mask = 0; mask < 16; mask++) {
  const ids = artworks.filter(art => mask & (1 << (Number(art.artwork_id) - 1))).map(art => art.artwork_id);
  const chosen = selectedArtworks(artworks, ids);
  const markup = courseSelection(artworks, ids);
  assert.equal((markup.match(/type="checkbox"/g) || []).length, 4);
  assert.equal((markup.match(/ checked/g) || []).length, chosen.length);
  assert.equal((markup.match(/class="route-artwork-id"/g) || []).length, chosen.length);
  const routeIds = [...markup.matchAll(/class="route-artwork-id">(\d+)</g)].map(match => match[1]);
  assert.deepEqual(routeIds, chosen.map(art => art.artwork_id));
  assert(markup.includes(`${chosen.length} / 4개 선택`));
}
const unselected = courseSelection(artworks, []);
assert(unselected.includes('선택한 작품이 없습니다.'));
assert(unselected.includes('data-action="select-all-artworks" >전체선택'));
const all = courseSelection(artworks, artworks.map(art => art.artwork_id));
assert(all.includes('data-action="select-all-artworks" disabled'));
assert(!all.includes('추천 코스') && !all.includes('키워드 코스'));
const malicious = courseSelection([{...artworks[0], artwork_title: '<img src=x onerror=alert(1)>', description: '<script>x</script>'}], []);
assert(!malicious.includes('<img') && !malicious.includes('<script>'));
assert(malicious.includes('&lt;img') && malicious.includes('&lt;script&gt;'));
console.log('작품 체크 목록 검증: 16개 선택 조합, 오름차순 경로, 전체선택 및 안전한 텍스트 출력');
