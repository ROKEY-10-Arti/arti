// 실제 브라우저를 열지 않고 앱의 이벤트·API·자동 복귀를 함께 검사합니다.
import assert from 'node:assert/strict';
const tick = () => new Promise(resolve => setImmediate(resolve));
async function checkFlow(manual, devMode = false) {
  const originals = new Map();
  const install = (name, value) => {originals.set(name, Object.getOwnPropertyDescriptor(globalThis, name)); Object.defineProperty(globalThis, name, {configurable:true, writable:true, value});};
  const expectedWrites = devMode ? [{status:'active'}] : [];
  const nativeNow = Date.now;
  let time=0, timerId=0;
  const timers=new Map(), sources=[], documentEvents=new Map(), windowEvents=new Map(), writes=[];
  let artworkRequests = 0;
  const root={innerHTML:'', querySelectorAll:()=>[]};
  const overlay={innerHTML:'', firstElementChild:null, querySelector:()=>({focus(){}})};
  const stub={focus(){},classList:{add(){},remove(){},toggle(){}},textContent:'',querySelector:()=>null};
  const storage=new Map([['arti-preparation','session-handoff']]);
  const session={id:'session-handoff', mode:'course', courseId:null, artworkIds:['art-01'], visitedIds:[], status:'calling', robotId:'DEV-ARTI'};
  class Source {
    constructor(url){this.url=url;this.handlers={};sources.push(this);}
    addEventListener(key,fn){this.handlers[key]=fn;}
    close(){this.closed=true;}
  }
  install('document',{hidden:false, activeElement:null, body:stub, querySelector:selector=>{
    if(selector==='#app')return root;
    if(selector==='#overlay')return overlay;
    if(['#main','#announcer','#toast','h1','[data-action="accept-terms"]'].includes(selector))return stub;
    return null;
  },querySelectorAll:()=>[],getElementById:()=>null,addEventListener:(key,fn)=>documentEvents.set(key,fn),removeEventListener:key=>documentEvents.delete(key)});
  install('window',{scrollTo(){},addEventListener:(key,fn)=>windowEvents.set(key,fn)});
  install('sessionStorage',{getItem:key=>storage.get(key)||null,setItem:(key,value)=>storage.set(key,value),removeItem:key=>storage.delete(key)});
  install('EventSource',Source);
  install('setTimeout',(fn,delay)=>{timers.set(++timerId,{fn,delay});return timerId;});
  install('clearTimeout',id=>timers.delete(id));
  install('setInterval',()=>++timerId); install('clearInterval',()=>{});
  install('fetch',async (url,options={})=>{
    if(options.method==='PATCH') {
      writes.push(JSON.parse(options.body));
      return {ok:true,json:async()=>({...session,status:'active',startedAt:123})};
    }
    assert.notEqual(url, '/api/catalog');
    if (url === '/api/artworks') artworkRequests++;
    const data=url==='/api/artworks'?{artworks:[{artwork_id:'001',artwork_title:'artwork1',artist_name:'artist1',theme:'theme1',description:'description1',location:'location1',image_path:'/static/art/art-001.png'}]}
      :url==='/api/auth/status'?{authenticated:false}:url==='/api/test-mode'?{enabled:false}:session;
    return {ok:true,json:async()=>data};
  });
  Date.now=()=>time;
  try {
    await import(`../static/js/app.js?handoff=${manual?'manual':'automatic'}-${devMode}`);
    await tick(); await tick();
    assert.equal(artworkRequests, 0, 'Restoring a visit does not fetch artwork data');
    const source=sources.find(source=>source.url.includes('/preparation/events'));
    assert(source);
    source.handlers.preparation({data:JSON.stringify({session_id:session.id,phase:devMode?'ready':'touring',message:'관람 시작',robot_id:devMode?'DEV-ARTI':'robot4',progress:1,dev_mode:devMode})});
    const click=async (action, dataset={})=>{const button={dataset:{action, ...dataset},disabled:false};await documentEvents.get('click')({target:{closest:()=>button}});};
    assert(!root.innerHTML.includes('data-action="start-tour"'));
    await tick();
    assert(root.innerHTML.includes('즐거운 관람 되세요!'));
    assert(!root.innerHTML.includes('지금, 이 작품') && !root.innerHTML.includes('description1'));
    assert.equal((root.innerHTML.match(/<button/g)||[]).length,1);
    assert.deepEqual(writes,expectedWrites);
    assert.equal(storage.has('arti-preparation'),false);
    const pending=[...timers.values()].find(timer=>timer.delay===10000);
    assert(pending);
    if(manual) {time=2000;await click('reset');assert(![...timers.values()].some(timer=>timer===pending));}
    else {time=10000;pending.fn();}
    assert(root.innerHTML.includes('page-home'));
    assert.equal(artworkRequests, 0, 'Home does not fetch artwork data');
    assert.deepEqual(writes,expectedWrites, '첫 화면 복귀는 관람 완료/취소나 가짜 방문 기록을 저장하지 않습니다.');
    await click('exhibition');
    await tick();
    assert.equal(artworkRequests, 1);
    assert.match(root.innerHTML, /data-action="view-art" data-id="001"/);
    assert.match(root.innerHTML, /<h3>artwork1<\/h3>/);
    await click('view-art', {id:'001'});
    assert.match(overlay.innerHTML, /<h2>artwork1<\/h2>/);
    assert.match(overlay.innerHTML, /<p class="artwork-artist">artist1<\/p>/);
    assert.match(overlay.innerHTML, /<p>description1<\/p>/);
    await click('close-modal');
    await click('back');
    assert(![...timers.values()].some(timer => timer.delay === 3000), 'Leaving the gallery stops artwork polling');
    const availability = sources.find(source => source.url === '/api/availability/stream');
    availability.handlers.availability({data:JSON.stringify({status:'available',available:true})});
    await click('begin-visit');
    await click('continue-usage');
    documentEvents.get('change')({target:{id:'terms-consent',checked:true,matches:()=>false}});
    await click('accept-terms');
    assert.equal(artworkRequests, 1, 'Usage, terms and mode selection do not request artwork data');
    await click('choose-mode', {mode:'course'});
    await tick();
    assert.equal(artworkRequests, 2);
    assert.match(root.innerHTML, /artwork1/);
    await click('back');
    await click('choose-mode', {mode:'free'});
    await tick();
    assert.equal(artworkRequests, 3);
    assert.match(root.innerHTML, /data-action="call"\s*>자유 관람/);
  } finally {
    windowEvents.get('pagehide')?.();
    Date.now=nativeNow;
    for(const [name,descriptor] of originals) {if(descriptor)Object.defineProperty(globalThis,name,descriptor);else delete globalThis[name];}
  }
}
await checkFlow(false);
await checkFlow(true, true);
console.log('앱 흐름 검증: 준비 완료 → 관람 시작 → 마지막 화면 → 10초/수동 복귀, 관람 중 기록 유지');
