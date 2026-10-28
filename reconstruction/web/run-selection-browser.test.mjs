import assert from 'node:assert/strict';
import {createServer} from 'node:http';
import {readFile} from 'node:fs/promises';
import path from 'node:path';
import {fileURLToPath} from 'node:url';
const {chromium}=await import(process.env.PLAYWRIGHT_MODULE || 'playwright');
const root=path.dirname(fileURLToPath(import.meta.url));
const makeJob=(id,manual)=>({id,status:'completed',created_at:'2026-10-28T00:00:00Z',label:manual?'Old experiment label':'Automatic reconstruction',method:'mvs',quality:'full',scene_specific_fit:manual,metrics:{},artifacts:[],warnings:[],stages:[]});
const manual=makeJob('manual-new',true),automatic=makeJob('automatic-old',false);
const fixture={jobs:[manual,automatic],details:[]};
const server=createServer(async(req,res)=>{
 const url=new URL(req.url,'http://localhost');
 const json=value=>{res.writeHead(200,{'Content-Type':'application/json','Cache-Control':'no-store'});res.end(JSON.stringify(value))};
 if(url.pathname==='/api/health')return json({status:'ready',engines:{colmap:{available:true},openmvs:{available:true}}});
 if(url.pathname==='/api/scans')return json({scans:[]});
 if(url.pathname==='/api/jobs')return json({jobs:fixture.jobs});
 if(url.pathname.startsWith('/api/jobs/')){const id=decodeURIComponent(url.pathname.slice('/api/jobs/'.length));fixture.details.push(id);const job=fixture.jobs.find(j=>j.id===id);if(job)return json({job});res.writeHead(404);return res.end('{}')}
 const relative=url.pathname==='/'?'index.html':decodeURIComponent(url.pathname).slice(1);
 if(relative.includes('..')){res.writeHead(403);return res.end()}
 try{const bytes=await readFile(path.join(root,relative));res.writeHead(200,{'Content-Type':({'.html':'text/html','.mjs':'text/javascript','.js':'text/javascript','.css':'text/css','.svg':'image/svg+xml'})[path.extname(relative)]||'application/octet-stream'});res.end(bytes)}catch{res.writeHead(404);res.end()}
});
await new Promise(r=>server.listen(0,'127.0.0.1',r));let browser;
try{
 browser=await chromium.launch({headless:true,...(process.env.CHROMIUM_PATH?{executablePath:process.env.CHROMIUM_PATH}:{}),args:['--enable-unsafe-swiftshader']});
 const page=await browser.newPage();const errors=[];page.on('pageerror',e=>errors.push(e.message));
 const selected=id=>page.locator(`[data-job-id="${id}"][aria-pressed="true"]`);
 await page.goto(`http://127.0.0.1:${server.address().port}/`);await selected('automatic-old').waitFor();
 assert.equal(fixture.details.includes('manual-new'),false,'newest scene-specific run must not be fetched as the automatic default');
 assert.match(await page.locator('[data-job-id="manual-new"]').textContent(),/Scene-specific fit · inferred/);
 await page.locator('[data-job-id="manual-new"]').click();await selected('manual-new').waitFor();
 await page.locator('#refresh').click();await selected('manual-new').waitFor();
 await page.reload();await selected('manual-new').waitFor();
 assert.equal(await page.evaluate(()=>localStorage.getItem('reconstruction-job-explicit')),'manual-new','an explicit experiment selection survives reload');
 await page.evaluate(()=>{localStorage.setItem('reconstruction-job','manual-new');localStorage.removeItem('reconstruction-job-explicit')});
 await page.reload();await selected('automatic-old').waitFor();
 await page.evaluate(()=>{localStorage.setItem('reconstruction-job','missing-run');localStorage.setItem('reconstruction-job-explicit','missing-run')});
 await page.reload();await selected('automatic-old').waitFor();
 fixture.jobs=[manual];await page.locator('#refresh').click();await page.waitForFunction(()=>document.querySelectorAll('.run-item').length===1&&!document.querySelector('.run-item[aria-pressed="true"]'));
 assert.equal(await page.locator('#stage-panel').isHidden(),true,'an experiment-only history needs an explicit selection');
 await page.locator('[data-job-id="manual-new"]').click();await selected('manual-new').waitFor();
 await page.locator('#refresh').click();await selected('manual-new').waitFor();
 assert.deepEqual(errors,[]);
 console.log(JSON.stringify({test:'run selection browser',automatic_default:'newest non scene-specific',explicit_experiment:'selectable and persistent',legacy_experiment_default:'migrated',missing_selection:'safe fallback',experiment_only_history:'requires explicit selection',hardware_motion:false}));
}finally{await browser?.close();await new Promise(r=>server.close(r))}
