// One in-flight command and one replaceable latest command, with a session lease.
export class WiFiLink {
  constructor(onMessage, onError) {
    this.onMessage=onMessage; this.onError=onError;
    this.token=null; this.pending=null; this.busy=false; this.closed=false;
    this.epoch=0; this.controller=null;
  }
  async request(path, body, token=this.token, timeout=300, keepalive=false) {
    const controller=new AbortController();
    const timer=setTimeout(()=>controller.abort(),timeout);
    if (path==='/api/command') this.controller=controller;
    try {
      const response=await fetch(path,{
        method:path==='/api/info'?'GET':'POST',headers:{'Content-Type':'text/plain',...(token?{'X-Rover-Token':token}:{})},
        body:path==='/api/info'?undefined:body,cache:'no-store',signal:controller.signal,keepalive,
      });
      const message=await response.json();
      if (!response.ok) throw new Error(message.error || 'Rover stopped or control session expired. Reconnect to continue.');
      if (message.protocol!=='rover-wifi-v1') throw new Error('Incompatible Wi-Fi firmware.');
      // The installed driver's channel names are opposite the physical sides.
      // Keep the UI in physical left/right coordinates, including its ACKs.
      if (Number.isInteger(message.left) && Number.isInteger(message.right))
        return {...message,left:message.right,right:message.left};
      return message;
    } finally {
      clearTimeout(timer);
      if (this.controller===controller) this.controller=null;
    }
  }
  send(line) {
    if (this.closed) return;
    if (line==='SYNC') return this.sync();
    if (line==='STOP') { void this.stop(); return; }
    if (!this.token) return;
    this.pending=line;
    if (!this.busy) void this.flush();
  }
  async sync() {
    await this.stop();
    if (this.closed) return;
    const epoch=++this.epoch;
    const bytes=crypto.getRandomValues(new Uint8Array(16));
    const client=Array.from(bytes,b=>b.toString(16).padStart(2,'0')).join('');
    try {
      const message=await this.request('/api/session',client,null,2000);
      if (this.closed || epoch!==this.epoch) {
        if (message.token) void this.request('/api/stop','',message.token,300,true).catch(()=>{});
        return;
      }
      if (!/^[0-9a-f]{32}$/.test(message.token || '')) throw new Error('Invalid rover session.');
      this.token=message.token; this.onMessage(message);
    } catch (error) { if (!this.closed && epoch===this.epoch) this.onError(error); }
  }
  async flush() {
    this.busy=true;
    try {
      while (this.pending!==null && !this.closed && this.token) {
        const line=this.pending; this.pending=null;
        const epoch=this.epoch;
        try {
          const parts=line.split(' ');
          const wireCommand=`M ${parts[1]} ${parts[3]} ${parts[2]}`;
          const message=await this.request('/api/command',wireCommand);
          if (epoch===this.epoch && !this.closed) this.onMessage(message);
        } catch (error) {
          if (epoch===this.epoch && !this.closed) { this.pending=null; this.onError(error); }
          break;
        }
      }
    } finally {
      this.busy=false;
      if (this.pending!==null && !this.closed && this.token) void this.flush();
    }
  }
  async stop() {
    ++this.epoch; this.pending=null; this.controller?.abort();
    const token=this.token;
    if (token) {
      try { await this.request('/api/stop','',token,300,true); } catch {}
    }
  }
  beaconStop() {
    this.pending=null;
    if (this.token) navigator.sendBeacon('/api/stop',new Blob([this.token],{type:'text/plain'}));
  }
  async close() {
    this.closed=true;
    await this.stop(); this.token=null;
  }
}
