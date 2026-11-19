// 초기 데이터는 data/catalog.json → SQLite → Flask API 순서로 전달됩니다.
export const artworks = [];
export const courses = [];

let catalogSignature = '';
export async function loadCatalog({signal} = {}) {
  const response = await fetch('/api/catalog', {cache: 'no-cache', signal});
  if (!response.ok) throw new Error('작품 정보를 불러오지 못했습니다.');
  const data = await response.json();
  const signature = JSON.stringify(data);
  if (signature === catalogSignature) return false;
  artworks.splice(0, artworks.length, ...data.artworks);
  courses.splice(0, courses.length, ...(data.courses || []));
  catalogSignature = signature;
  return true;
}

export const getArtwork = id => artworks.find(art => art.artwork_id === id);
export const artImage = art => art.image_path;
// 관람 세션, ROS, 평면도에서 사용하는 기존 ID 형식입니다.
export const tourArtworkId = id => `art-${String(Number(id)).padStart(2, '0')}`;
export const catalogArtworkId = id => id.slice(4).padStart(3, '0');
