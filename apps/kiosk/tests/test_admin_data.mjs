import assert from 'node:assert/strict';
import {artworkPayload, logRows, numericValue} from '../static/js/admin-data.js';
const inline = artworkPayload({artwork_title:'new title',artist_name:'new artist',location:'007',theme:'theme',description:'text',x:'0',y:''});
assert.deepEqual(inline,{artwork_title:'new title',artist_name:'new artist',location:'007',theme:'theme',description:'text',x:0,y:null});
assert.deepEqual(artworkPayload({artwork_title:'Artwork title',image_alt:'',x:'-2.5',y:'3'}),{artwork_title:'Artwork title',image_alt:'',x:-2.5,y:3});
for (const value of ['NaN','Infinity','oops','1e13']) assert.throws(()=>numericValue(value));
const malicious='<img src=x onerror="alert(1)">';
const rows=logRows([{log_id:7,robot_id:malicious,sent_at:0,message:malicious,operator:{
  sender:'키오스크',target:'로봇 1',namespace:'/robot1',resource:'/robot1/arti/start_tour',
  type:'ROS 액션 · 진행 상태',payload:malicious,description:'1번 작품 설명 중',
}}], new Set(['log-detail-7']));
assert(!rows.includes('<img')); assert(rows.includes('&lt;img')); assert(rows.includes('id="log-detail-7" open'));
assert(rows.includes('1번 작품 설명 중')); assert(rows.includes('/robot1/arti/start_tour'));
assert.equal((rows.match(/<td>/g)||[]).length,4);
assert(logRows([]).includes('조건에 맞는'));
console.log('관리자 DB 표 검증: 작품 입력 변환, 숫자 검증, 로그 출력');
