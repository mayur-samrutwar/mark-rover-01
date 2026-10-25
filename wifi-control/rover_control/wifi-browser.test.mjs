import assert from 'node:assert/strict';
import {directionName} from './controls.mjs';
const {chromium}=await import(process.env.PLAYWRIGHT_MODULE || 'playwright');
const browser=await chromium.launch({headless:true,...(process.env.CHROMIUM_PATH?{executablePath:process.env.CHROMIUM_PATH}:{})});
const page=await browser.newPage({viewport:{width:1440,height:1120}});
let token='', armed=false, lastSequence=0, left=0, right=0, counter=0;
let suppress=false, infoDelay=0, infoErrors=0;
const requests=[], errors=[];
const snapshot=type=>({protocol:'rover-wifi-v1',type,ready:true,armed,seq:lastSequence,left,right});
page.on('pageerror',e=>errors.push(e.message));
await page.addInitScript(()=>{ document.hasFocus=()=>true; });
await page.route('**/api/**',async route=>{
  const request=route.request(), url=new URL(request.url()), body=request.postData() || '';
  requests.push({path:url.pathname,body});
  const respond=(message,status=200)=>route.fulfill({status,contentType:'application/json',body:JSON.stringify(message)});
  if (url.pathname==='/api/info') {
    if (infoErrors>0) { --infoErrors; return respond({error:'Temporary network error'},502); }
    if (infoDelay) await new Promise(resolve=>setTimeout(resolve,infoDelay));
    return respond(snapshot('info'));
  }
  if (url.pathname==='/api/session') {
    if (armed) return respond({error:'Another controller is connected'},409);
    token=(++counter).toString(16).padStart(32,'0'); armed=true; lastSequence=left=right=0;
    return respond({...snapshot('hello'),token});
  }
  if (request.headers()['x-rover-token']!==token) return respond({error:'Session expired'},403);
  if (url.pathname==='/api/stop') {armed=false;left=right=0;return respond(snapshot('stopped'));}
  if (url.pathname==='/api/command') {
    if (suppress) { await new Promise(resolve=>setTimeout(resolve,600)); return respond(snapshot('state')).catch(()=>{}); }
    const [,seq,l,r]=body.split(' ').map(Number);
    if (!armed || seq<=lastSequence) {armed=false;left=right=0;return respond(snapshot('fault'),409);}
    lastSequence=seq; left=l; right=r; return respond(snapshot('state'));
  }
  return respond({error:'Unknown'},404);
});
const expected=(l,r)=>page.waitForFunction(name=>
  document.getElementById('motion').textContent===name,directionName(l,r));
try {
  await page.goto(process.env.ROVER_URL || 'http://127.0.0.1:8765/');
  await page.getByRole('button',{name:'Connect',exact:true}).click();
  await page.waitForFunction(()=>document.getElementById('connection-state').textContent==='Connected');
  await expected(0,0);
  // A slow idle status response must not trigger the driving watchdog UI.
  infoDelay=650;
  await page.waitForResponse(response=>response.url().endsWith('/api/info'));
  infoDelay=0;
  assert.equal(await page.locator('#connection-state').textContent(),'Connected');
  assert.equal(armed,false,'Idle must leave the firmware motor lease disarmed');
  infoErrors=3;
  await page.waitForFunction(()=>document.getElementById('connection-state').textContent==='Reconnecting');
  await page.waitForFunction(()=>document.getElementById('connection-state').textContent==='Connected');
  assert.equal(armed,false,'Automatic idle recovery must never arm the motors');
  await page.keyboard.down('w'); await expected(128,128);
  await page.keyboard.up('w'); await expected(0,0);
  await page.keyboard.down('ArrowUp'); await expected(128,128);
  await page.keyboard.down('w'); await page.keyboard.up('ArrowUp'); await expected(128,128);
  await page.keyboard.up('w'); await expected(0,0);
  await page.keyboard.down('a'); await expected(-128,128);
  assert.match(requests.filter(r=>r.path==='/api/command').at(-1).body,/^M \d+ 128 -128$/);
  await page.keyboard.up('a'); await expected(0,0);
  await page.keyboard.down('d'); await expected(128,-128);
  assert.match(requests.filter(r=>r.path==='/api/command').at(-1).body,/^M \d+ -128 128$/);
  await page.keyboard.up('d'); await expected(0,0);
  await page.keyboard.down('s'); await expected(-128,-128);
  await page.keyboard.down('d'); await expected(-43,-128);
  await page.keyboard.up('s'); await page.keyboard.up('d'); await expected(0,0);
  await page.keyboard.down('w'); await page.keyboard.down('s'); await expected(0,0);
  await page.keyboard.up('w'); await page.keyboard.up('s');
  await page.keyboard.down('w'); await expected(128,128);
  await page.evaluate(()=>window.dispatchEvent(new Event('blur'))); await expected(0,0);
  await page.keyboard.up('w');
  await page.keyboard.down('w'); await expected(128,128);
  await page.keyboard.press('Space');
  await page.waitForFunction(()=>document.getElementById('motion').textContent==='Stopped / locked');
  await page.keyboard.up('w');
  assert.ok(requests.some(r=>r.path==='/api/stop'));
  await page.getByRole('button',{name:'Resume controls'}).click();
  await page.waitForFunction(()=>document.getElementById('connection-state').textContent==='Connected');
  await expected(0,0);
  suppress=true;
  await page.keyboard.down('w');
  await page.waitForFunction(()=>document.getElementById('motion').textContent==='Stopped / locked');
  await page.keyboard.up('w');
  suppress=false;
  await page.getByRole('button',{name:'Resume controls'}).click();
  await page.waitForFunction(()=>document.getElementById('connection-state').textContent==='Connected');
  await expected(0,0);
  await page.getByRole('button',{name:'Disconnect'}).click();
  await page.waitForFunction(()=>document.getElementById('connection-state').textContent==='Disconnected');
  assert.equal(armed,false); assert.equal(left,0); assert.equal(right,0);
  assert.deepEqual(errors,[]);
  await page.screenshot({path:'/tmp/rover-wifi-desktop.png',fullPage:true});
  await page.setViewportSize({width:390,height:844});
  assert.equal(await page.evaluate(()=>document.documentElement.scrollWidth>innerWidth),false);
  console.log('Wi-Fi browser tests passed: session, WASD, arcs, release, opposing keys, blur, emergency, resume, request timeout, disconnect and mobile layout. No real hardware used.');
} finally {await browser.close();}
