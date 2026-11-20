import assert from 'node:assert/strict';
import {artworkPayload, posePayload, logRows, robotRows, numericValue} from '../static/js/admin-data.js';
const inline = artworkPayload({artwork_title:'new title',artist_name:'new artist',location:'007',theme:'theme',description:'text',x:'0',y:''});
assert.deepEqual(inline,{artwork_title:'new title',artist_name:'new artist',location:'007',theme:'theme',description:'text',x:0,y:null});
assert.deepEqual(artworkPayload({artwork_title:'Artwork title',image_alt:'',x:'-2.5',y:'3'}),{artwork_title:'Artwork title',image_alt:'',x:-2.5,y:3});
for (const value of ['NaN','Infinity','oops','1e13']) assert.throws(()=>numericValue(value));
assert.deepEqual(posePayload({x:'0',y:'1',theta:'-1.2',frame_id:'map'},true),{active:true,frame_id:'map',pose:{x:0,y:1,theta:-1.2}});
assert.deepEqual(posePayload({x:'',y:'',theta:'',frame_id:'map'},false),{active:false,frame_id:'map',pose:null});
assert.throws(()=>posePayload({x:'1',y:'',theta:'',frame_id:'map'},true));
const malicious='<img src=x onerror="alert(1)">';
const rows=logRows([{log_id:7,robot_id:malicious,sent_at:0,message:malicious,operator:{
  sender:'키오스크',target:'로봇 1',namespace:'/robot1',resource:'/robot1/arti/start_tour',
  type:'ROS 액션 · 진행 상태',payload:malicious,description:'1번 작품 설명 중',
}}], new Set(['log-detail-7']));
assert(!rows.includes('<img')); assert(rows.includes('&lt;img')); assert(rows.includes('id="log-detail-7" open'));
assert(rows.includes('1번 작품 설명 중')); assert(rows.includes('/robot1/arti/start_tour'));
assert.equal((rows.match(/<td>/g)||[]).length,4);
const poseRows=robotRows([{robot_id:malicious,pose:{x:0,y:1,theta:-2},active:true,frame_id:malicious,pose_updated_at:0}]);
assert(!poseRows.includes('<img')); assert(poseRows.includes('&lt;img')); assert(poseRows.includes('<td>0</td>'));
assert(logRows([]).includes('조건에 맞는')); assert(robotRows([]).includes('등록된 로봇'));
console.log('관리자 DB 표 검증: 입력 변환·숫자 검증·위치 수정·로그 및 로봇 값 안전한 출력');
