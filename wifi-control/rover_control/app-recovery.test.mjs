import test from 'node:test';
import assert from 'node:assert/strict';
import {readFileSync} from 'node:fs';
import {DRIVE_KEYS,mixKeys,directionName} from './controls.mjs';
import {WiFiLink} from './wifi-link.mjs';

const source=readFileSync(new URL('./app.mjs',import.meta.url),'utf8').replace(/^import .*;\n/gm,'');
const settle=async()=>{ for (let i=0;i<4;i++) await new Promise(resolve=>setImmediate(resolve)); };
const deferred=()=>{ let resolve; const promise=new Promise(r=>{resolve=r;}); return {promise,resolve}; };

function fixture(t) {
  class Target {
    constructor() {
      this.listeners=new Map(); this.textContent=''; this.value='128'; this.disabled=false; this.hidden=false;
      this.dataset={}; this.style={setProperty(){}}; this.classList={toggle(){}};
    }
    addEventListener(name,fn) { const handlers=this.listeners.get(name)||[]; handlers.push(fn); this.listeners.set(name,handlers); }
    fire(name,event={}) {
      const full={target:this,preventDefault(){},repeat:false,...event};
      return Promise.all((this.listeners.get(name)||[]).map(fn=>fn(full)));
    }
    closest() { return null; }
    matches() { return false; }
    setPointerCapture() {}
  }
  const elements=new Map(), element=id=>{
    if (!elements.has(id)) elements.set(id,new Target());
    return elements.get(id);
  };
  const buttons=['KeyW','KeyA','KeyS','KeyD'].map(code=>{ const button=new Target(); button.dataset.code=code; return button; });
  const document=new Target(); document.body=new Target(); document.hidden=false;
  document.getElementById=element; document.querySelectorAll=()=>buttons; document.hasFocus=()=>true;
  const window=new Target();
  let now=0, nextTimer=0;
  const intervals=[], timers=new Map();
  const tick=async(ms=100)=>{
    now+=ms;
    for (const [id,timer] of [...timers]) if (timer.at<=now) {timers.delete(id);timer.fn();}
    intervals.forEach(fn=>fn()); await settle();
  };
  const requests=[], responses={info:[],session:[],command:[],stop:[]};
  let armed=false, seq=0, left=0, right=0, sessionCount=0;
  const state=type=>({protocol:'rover-wifi-v1',type,ready:true,armed,seq,left,right});
  t.mock.method(globalThis,'fetch',async(path,options)=>{
    requests.push({path,...options});
    const route=path.slice('/api/'.length), action=responses[route].shift();
    if (action instanceof Error) throw action;
    const override=await action;
    let message;
    if (route==='info') message=state('info');
    else if (route==='session') {
      armed=true; seq=left=right=0;
      message={...state('hello'),token:(++sessionCount).toString(16).padStart(32,'0')};
    } else if (route==='command') {
      [seq,left,right]=options.body.split(' ').slice(1).map(Number); message=state('state');
    } else if (route==='stop') {armed=false; left=right=0; message=state('stopped');}
    else throw new Error(`Unexpected endpoint ${path}`);
    return {ok:true,json:async()=>override || message};
  });
  const run=new Function('document','window','location','performance','setInterval','setTimeout','clearTimeout','console','WiFiLink','DRIVE_KEYS','mixKeys','directionName',source);
  run(document,window,{hostname:'test.invalid'},{now:()=>now},fn=>intervals.push(fn),
    (fn,ms)=>{const id=++nextTimer;timers.set(id,{fn,at:now+ms});return id;},id=>timers.delete(id),
    {info(){},warn(){}},WiFiLink,DRIVE_KEYS,mixKeys,directionName);
  return {element,document,window,buttons,tick,requests,responses,
    connect:async()=>{await element('connect').fire('click');await settle();},
    press:async code=>{await document.fire('keydown',{code});await settle();},
    release:async code=>{await document.fire('keyup',{code});await settle();},
    count:route=>requests.filter(request=>request.path===`/api/${route}`).length,
    title:()=>element('connection-state').textContent,
  };
}

test('failed initial handshake recovers with read-only health checks and requires a fresh input',async t=>{
  const app=fixture(t);
  app.responses.session.push(new Error('offline'));
  await app.connect();
  assert.equal(app.title(),'Reconnecting');
  assert.equal(app.element('resume').hidden,true);
  assert.ok(app.buttons.every(button=>button.disabled));
  await app.press('KeyW');
  assert.equal(app.count('command'),0);
  await app.tick();
  assert.equal(app.title(),'Connected');
  assert.equal(app.count('session'),1,'Health recovery must not acquire a motor session');
  assert.equal(app.count('command'),0);
  await app.document.fire('keydown',{code:'KeyW',repeat:true});
  await app.tick();
  assert.equal(app.count('command'),0,'A direction held during recovery must not resume');
  await app.release('KeyW'); await app.press('KeyW');
  assert.equal(app.count('session'),2);
  assert.equal(app.count('command'),1);
  assert.match(app.requests.filter(request=>request.path==='/api/command').at(-1).body,/^M 1 -128 -128$/);
  await app.release('KeyW');
});

test('driving failure discards held keys and only restores idle controls',async t=>{
  const app=fixture(t);
  await app.connect();
  app.responses.command.push(new Error('Wi-Fi lost'));
  await app.press('KeyW');
  assert.equal(app.title(),'Reconnecting');
  assert.equal(app.element('motion').textContent,'Stopped');
  const sessionCount=app.count('session'), commandCount=app.count('command');
  assert.ok(app.count('stop')>=2);
  await app.tick();
  assert.equal(app.title(),'Connected');
  await app.tick(1000);
  assert.equal(app.count('session'),sessionCount);
  assert.equal(app.count('command'),commandCount);
  await app.document.fire('keydown',{code:'KeyW',repeat:true}); await settle();
  assert.equal(app.count('command'),commandCount);
  await app.release('KeyW'); await app.press('KeyD');
  assert.equal(app.count('command'),commandCount+1);
  await app.release('KeyD');
});

test('busy, unavailable and incompatible health responses cannot enable drive',async t=>{
  const app=fixture(t);
  app.responses.session.push(new Error('offline'));
  await app.connect();
  for (const response of [
    {protocol:'rover-wifi-v1',type:'info',ready:true,armed:true,left:0,right:0},
    {protocol:'rover-wifi-v1',type:'info',ready:false,armed:false,left:0,right:0},
    {protocol:'other',type:'info',ready:true,armed:false,left:0,right:0},
    {protocol:'rover-wifi-v1',type:'info',ready:true,armed:false,left:15,right:0},
  ]) {
    app.responses.info.push(response);
    await app.tick(1000);
    assert.equal(app.title(),'Reconnecting');
    assert.ok(app.buttons.every(button=>button.disabled));
    assert.equal(app.count('session'),1);
  }
  await app.tick(1000);
  assert.equal(app.title(),'Connected');
  assert.equal(app.count('session'),1);
});

test('operator emergency stays latched across Wi-Fi recovery until explicit Resume',async t=>{
  const app=fixture(t);
  await app.connect();
  await app.press('Escape');
  assert.equal(app.title(),'Stopped / locked');
  assert.equal(app.element('resume').hidden,false);
  const polls=app.count('info'), sessions=app.count('session');
  await app.tick(5000); await app.press('KeyW');
  assert.equal(app.title(),'Stopped / locked');
  assert.equal(app.count('info'),polls);
  assert.equal(app.count('session'),sessions);
  await app.element('resume').fire('click'); await settle();
  assert.equal(app.title(),'Connected');
  assert.equal(app.count('command'),0);
});

test('a timed-out pointer handshake drops the held direction and ignores its late hello',async t=>{
  const app=fixture(t);
  await app.connect();
  const hello=deferred(); app.responses.session.push(hello.promise);
  const forward=app.buttons[0];
  await forward.fire('pointerdown',{button:0,pointerId:7}); await settle();
  app.responses.info.push(new Error('still offline'));
  await app.tick(2501);
  assert.equal(app.title(),'Reconnecting');
  hello.resolve(); await settle();
  assert.equal(app.count('command'),0,'Late session hello must not restore the held pointer');
  await app.tick(1000);
  assert.equal(app.title(),'Connected');
  const sessions=app.count('session');
  await app.tick(1000);
  assert.equal(app.count('session'),sessions);
  assert.equal(app.count('command'),0);
  await forward.fire('pointerup',{pointerId:7});
  await forward.fire('pointerdown',{button:0,pointerId:8}); await settle();
  assert.equal(app.count('command'),1);
  await forward.fire('pointerup',{pointerId:8}); await settle();
});

test('the 500 ms driving ACK cutoff stops the lease and late ACKs cannot revive movement',async t=>{
  const app=fixture(t);
  await app.connect();
  const ack=deferred(); app.responses.command.push(ack.promise);
  await app.press('KeyW');
  await app.tick(501);
  assert.equal(app.title(),'Reconnecting');
  const commands=app.count('command'), sessions=app.count('session');
  await app.tick();
  assert.equal(app.title(),'Connected');
  ack.resolve(); await settle();
  assert.equal(app.element('motion').textContent,'Stopped');
  await app.tick(1000);
  assert.equal(app.count('command'),commands);
  assert.equal(app.count('session'),sessions);
});

test('an idle response from a discarded connection cannot overwrite or unblock a newer poll',async t=>{
  const app=fixture(t);
  await app.connect();
  const oldPoll=deferred(); app.responses.info.push(oldPoll.promise);
  await app.tick();
  await app.connect(); // Disconnect the first link while its read-only poll is outstanding.
  app.responses.session.push(new Error('second link offline'));
  await app.connect();
  const newPoll=deferred(); app.responses.info.push(newPoll.promise);
  await app.tick(1000);
  const polls=app.count('info');
  oldPoll.resolve({protocol:'rover-wifi-v1',type:'info',ready:true,armed:false,left:0,right:0});
  await settle(); await app.tick(1000);
  assert.equal(app.title(),'Reconnecting');
  assert.equal(app.count('info'),polls,'Old finally must not clear the newer in-flight poll');
  newPoll.resolve({protocol:'rover-wifi-v1',type:'info',ready:true,armed:false,left:0,right:0});
  await settle();
  assert.equal(app.title(),'Connected');
  assert.equal(app.count('command'),0);
});

test('an idle response begun before operator emergency cannot unlock controls',async t=>{
  const app=fixture(t);
  await app.connect();
  const poll=deferred(); app.responses.info.push(poll.promise);
  await app.tick(); await app.press('Space');
  poll.resolve({protocol:'rover-wifi-v1',type:'info',ready:true,armed:false,left:0,right:0});
  await settle(); await app.tick(1000);
  assert.equal(app.title(),'Stopped / locked');
  assert.ok(app.buttons.every(button=>button.disabled));
  assert.equal(app.count('command'),0);
});
