import test from 'node:test';
import assert from 'node:assert/strict';
import {mixKeys} from './controls.mjs';
import {WiFiLink} from './wifi-link.mjs';

function mockRover(t) {
  const token='0123456789abcdef0123456789abcdef';
  const requests=[], messages=[], errors=[];
  let armed=false, seq=0, left=0, right=0;
  const state=type=>({protocol:'rover-wifi-v1',type,ready:true,armed,seq,left,right});
  t.mock.method(globalThis,'fetch',async (path,options)=>{
    requests.push({path,...options});
    let message;
    if (path==='/api/info') message=state('info');
    else if (path==='/api/session') {
      armed=true; seq=left=right=0;
      message={...state('hello'),token};
    } else {
      assert.equal(options.headers['X-Rover-Token'],token);
      if (path==='/api/command') {
        assert.equal(armed,true);
        const parts=options.body.split(' ');
        assert.equal(parts[0],'M');
        [seq,left,right]=parts.slice(1).map(Number);
        message=state('state');
      } else if (path==='/api/stop') {
        armed=false; left=right=0;
        message=state('stopped');
      } else throw new Error(`Unexpected endpoint ${path}`);
    }
    return {ok:true,json:async()=>message};
  });
  const link=new WiFiLink(message=>messages.push(message),error=>errors.push(error));
  return {link,requests,messages,errors};
}

async function drain(link) {
  for (let i=0;i<10 && (link.busy || link.pending!==null);i++)
    await new Promise(resolve=>setImmediate(resolve));
  assert.equal(link.busy,false);
  assert.equal(link.pending,null);
}

test('W/S change motor polarity while A/D retain their known working wire directions',async t=>{
  const {link,requests,messages,errors}=mockRover(t);
  await link.sync();
  const cases=[
    {keys:['KeyW'],physical:[128,128],wire:[-128,-128]},
    {keys:['KeyS'],physical:[-128,-128],wire:[128,128]},
    {keys:['KeyA'],physical:[-128,128],wire:[128,-128]},
    {keys:['KeyD'],physical:[128,-128],wire:[-128,128]},
  ];
  let seq=0;
  for (const {keys,physical,wire} of cases) {
    const mixed=mixKeys(new Set(keys),128);
    assert.deepEqual(mixed,physical);
    link.send(`M ${++seq} ${mixed[0]} ${mixed[1]}`);
    await drain(link);
    assert.equal(requests.at(-1).body,`M ${seq} ${wire[0]} ${wire[1]}`);
    assert.deepEqual([messages.at(-1).left,messages.at(-1).right],physical);
    assert.equal(messages.at(-1).seq,seq);
  }
  await link.close();
  assert.deepEqual(errors,[]);
});

test('asymmetric forward/reverse arcs keep physical channel order through wire and ACK',async t=>{
  const {link,requests,messages,errors}=mockRover(t);
  await link.sync();
  const cases=[
    {keys:['KeyW','KeyA'],physical:[43,128],wire:[-43,-128]},
    {keys:['KeyW','KeyD'],physical:[128,43],wire:[-128,-43]},
    {keys:['KeyS','KeyA'],physical:[-128,-43],wire:[128,43]},
    {keys:['KeyS','KeyD'],physical:[-43,-128],wire:[43,128]},
  ];
  let seq=0;
  for (const {keys,physical,wire} of cases) {
    assert.deepEqual(mixKeys(new Set(keys),128),physical);
    link.send(`M ${++seq} ${physical[0]} ${physical[1]}`);
    await drain(link);
    assert.equal(requests.at(-1).body,`M ${seq} ${wire[0]} ${wire[1]}`);
    assert.deepEqual([messages.at(-1).left,messages.at(-1).right],physical);
  }
  await link.close();
  assert.deepEqual(errors,[]);
});

test('full manual range and zero remain exact; STOP remains a separate authenticated request',async t=>{
  const {link,requests,messages,errors}=mockRover(t);
  await link.sync();
  assert.deepEqual([messages.at(-1).left,messages.at(-1).right],[0,0]);
  link.send('M 1 255 -255');
  await drain(link);
  assert.equal(requests.at(-1).body,'M 1 -255 255');
  assert.deepEqual([messages.at(-1).left,messages.at(-1).right],[255,-255]);
  link.send('M 2 0 0');
  await drain(link);
  assert.equal(requests.at(-1).body,'M 2 0 0');
  assert.deepEqual([messages.at(-1).left,messages.at(-1).right],[0,0]);
  link.send('STOP');
  await new Promise(resolve=>setImmediate(resolve));
  assert.equal(requests.at(-1).path,'/api/stop');
  assert.equal(requests.at(-1).body,'');
  assert.equal(requests.at(-1).keepalive,true);
  const stopped=await link.request('/api/info');
  assert.equal(stopped.armed,false);
  assert.deepEqual([stopped.left,stopped.right],[0,0]);
  await link.close();
  assert.deepEqual(errors,[]);
});
