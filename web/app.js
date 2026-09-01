'use strict';
/* ------------------------------------------------------------------ utils */
const $ = id => document.getElementById(id);
const fmt = n => (n ?? 0).toLocaleString('en-US');
const css = v => getComputedStyle(document.documentElement).getPropertyValue(v).trim();

function dec(b64, T){
  const s = atob(b64), u = new Uint8Array(s.length);
  for (let i = 0; i < s.length; i++) u[i] = s.charCodeAt(i);
  return new T(u.buffer);
}
function toRGB(s){
  s = s.trim();
  if (s[0] === '#'){
    if (s.length === 4) s = '#'+s[1]+s[1]+s[2]+s[2]+s[3]+s[3];
    return [parseInt(s.slice(1,3),16), parseInt(s.slice(3,5),16), parseInt(s.slice(5,7),16)];
  }
  const m = s.match(/[\d.]+/g); return [+m[0], +m[1], +m[2]];
}
function ramp256(stops){
  const out = new Array(256), n = stops.length-1, rgb = stops.map(toRGB);
  for (let i = 0; i < 256; i++){
    const t = i/255*n, k = Math.min(n-1, t|0), f = t-k, a = rgb[k], b = rgb[k+1];
    out[i] = `rgb(${Math.round(a[0]+(b[0]-a[0])*f)},${Math.round(a[1]+(b[1]-a[1])*f)},${Math.round(a[2]+(b[2]-a[2])*f)})`;
  }
  return out;
}
const NSH = 14;
function shades(cols){
  const rgb = cols.map(toRGB), out = new Array(cols.length*NSH);
  for (let c = 0; c < cols.length; c++) for (let q = 0; q < NSH; q++){
    const f = .42 + .70*(q/(NSH-1));
    out[c*NSH+q] = `rgb(${Math.min(255,rgb[c][0]*f|0)},${Math.min(255,rgb[c][1]*f|0)},${Math.min(255,rgb[c][2]*f|0)})`;
  }
  return out;
}
const SPECTRAL = ramp256(['#7A1E8C','#2A2FB4','#0B7FD6','#0FC3B4','#57D63A','#CFE01C','#FFA51C','#E32A1C']);
const CLASSC = ['#9A8C7A','#7A8899','#C05A5A','#E0A33E','#5D9B6B','#6F5FA8','#E0457B','#A0A6AD'];
const CLASSN = ['ground','road','building','pole','vegetation','car','pedestrian','other'];

/* what the detector itself did with each point. the two desaturated entries
   are the ones the network contributed nothing to: ground comes from the
   geometric remover, and "never clustered" never reached the network at all.
   the saturated ones are its actual output. */
const DETC = ['#8E9A86','#3E6E9C','#6F5FA8','#E0457B','#E0A33E','#B5654A','#CBD1D8'];
const DETN = ['ground (geometric)','examined, rejected','car','pedestrian',
              'cyclist','never clustered','no cell here'];
let SP = null, CL = null, TV = null, DT = null;
const pal = () => { if (!SP){ SP = shades(SPECTRAL); CL = shades(CLASSC);
  TV = shades([css('--ink3'), css('--no'), css('--ok')]); DT = shades(DETC); } };

/* ------------------------------------------------------------------ state */
const S = {
  job:null, frames:[], data:new Map(), cur:-1, playing:false, spf:600, last:0,
  src:'ztop', paint:'height', mesh:true, rings:true, camera:true, proj:true,
  ob:{az:3.90, el:.40, S:14, px0:0, py0:0, vex:2}, drag:null, es:null, touched:false,
};
const cv = $('view'), ctx = cv.getContext('2d');
let W = 0, H = 0, dpr = 1;
const QMAX = 1<<17;
const qk = new Int32Array(QMAX), qd = new Float32Array(QMAX),
      qo = new Int32Array(QMAX), qc = new Int32Array(1026);
const KS = new Int32Array(4);

const frameOf = f => S.data.get(f);
function tiersOf(f){
  const d = frameOf(f);
  if (!d) return null;
  if (!d._t) d._t = d.tiers.map(t => ({res:t.res, x0:t.x0, y0:t.y0, nx:t.nx, ny:t.ny,
    rin:t.rin, rout:t.rout, ztop:dec(t.ztop,Int16Array), zgnd:dec(t.zgnd,Int16Array),
    cls:dec(t.cls,Uint8Array), flag:dec(t.flag,Uint8Array),
    det:t.det ? dec(t.det,Uint8Array) : null}));
  return d._t;
}

/* --------------------------------------------------------------- renderer */
function basis(sc){
  const ca=Math.cos(S.ob.az), sa=Math.sin(S.ob.az),
        ce=Math.cos(S.ob.el), se=Math.sin(S.ob.el);
  return {ux:-sa*sc, uy:ca*sc, vx:se*ca*sc, vy:se*sa*sc, vz:-ce*sc*S.ob.vex, ca, sa, ce};
}
function fit(){
  const T = tiersOf(S.frames[S.cur]); if (!T || !W) return;
  const B = basis(1);
  let ax=1e9,bx=-1e9,ay=1e9,by=-1e9;
  for (const t of T) for (let j=0;j<t.ny;j+=3) for (let i=0;i<t.nx;i+=3){
    if (!(t.flag[j*t.nx+i] & 4)) continue;
    const X=t.x0+i*t.res, Y=t.y0+j*t.res;
    const a=X*B.ux+Y*B.uy;
    const b0=X*B.vx+Y*B.vy+t.zgnd[j*t.nx+i]*.001*B.vz;
    const b1=X*B.vx+Y*B.vy+t.ztop[j*t.nx+i]*.001*B.vz;
    if(a<ax)ax=a; if(a>bx)bx=a;
    if(b1<ay)ay=b1; if(b0>by)by=b0;
  }
  if (ax > bx) return;
  S.ob.S = Math.min(W*.9/(bx-ax||1), H*.9/(by-ay||1));
  S.ob.px0 = W/2 - (ax+bx)/2*S.ob.S;
  S.ob.py0 = H/2 - (ay+by)/2*S.ob.S;
  S.touched = false;
}
function draw(){
  pal();
  ctx.setTransform(dpr,0,0,dpr,0,0);
  ctx.fillStyle = css('--stage'); ctx.fillRect(0,0,W,H);
  const d = frameOf(S.frames[S.cur]), T = tiersOf(S.frames[S.cur]);
  if (!T){ $('tagL').hidden = $('tagB').hidden = true; return; }
  $('empty').style.display = 'none';

  const B = basis(S.ob.S), sc = S.ob.S;
  const {ux,uy,vx,vy,vz} = B, X0 = S.ob.px0, Y0 = S.ob.py0;
  const L0=-70,L1=W+70,T0=-70,T1=H+70;

  if (S.rings){
    ctx.strokeStyle = css('--line'); ctx.setLineDash([3,4]); ctx.lineWidth = 1;
    for (const r of [10,25,50]){
      ctx.beginPath();
      for (let q=0;q<=72;q++){ const w=q/72*6.2832, X=r*Math.cos(w), Y=r*Math.sin(w);
        const a=X*ux+Y*uy+X0, b=X*vx+Y*vy+Y0; q?ctx.lineTo(a,b):ctx.moveTo(a,b); }
      ctx.stroke();
    }
    ctx.setLineDash([]);
  }

  /* the wedge the camera actually sees. the photo is 23% of the map, and
     saying so in a caption is weaker than drawing the edge of it. */
  if (S.camera && d && d.cam){
    const R = 45, y0 = (d.cam.yaw - d.cam.fov/2)*Math.PI/180,
                  y1 = (d.cam.yaw + d.cam.fov/2)*Math.PI/180;
    const at = a => { const X=R*Math.cos(a), Y=R*Math.sin(a);
                      return [X*ux+Y*uy+X0, X*vx+Y*vy+Y0]; };
    ctx.strokeStyle = css('--mark'); ctx.lineWidth = 1.2;
    ctx.globalAlpha = .55; ctx.setLineDash([6,4]);
    ctx.beginPath();
    for (const a of [y0, y1]){ const p = at(a); ctx.moveTo(X0,Y0); ctx.lineTo(p[0],p[1]); }
    ctx.stroke();
    ctx.beginPath();
    for (let i=0;i<=32;i++){ const p = at(y0+(y1-y0)*i/32);
      i?ctx.lineTo(p[0],p[1]):ctx.moveTo(p[0],p[1]); }
    ctx.stroke();
    ctx.setLineDash([]);
    const mid = at((y0+y1)/2);
    ctx.fillStyle = css('--mark');
    ctx.font = '10px "IBM Plex Mono", monospace';
    ctx.textAlign = 'center';
    ctx.fillText(`camera ${d.cam.fov.toFixed(0)}\u00b0`, mid[0], mid[1] - 6);
    ctx.textAlign = 'start';
    ctx.globalAlpha = 1;
  }

  /* one stride shared by every tier so the 2:1 ratio between them survives */
  const budget = (S.drag||S.playing) ? 16000 : 40000;
  const base = T[0].res;
  let k = 1, qn = 0;
  while (base*k*sc < .8) k <<= 1;
  for(;;){
    qn = 0;
    for (let t=0;t<T.length;t++){
      KS[t]=k; const Tt=T[t];
      if (k>=Tt.nx || k>=Tt.ny) continue;
      const A = Tt[S.src], nx = Tt.nx;
      const rin = t ? Tt.rin - Tt.res*k*1.5 : -1, rout = Tt.rout;
      for (let j=0;j+k<Tt.ny;j+=k){
        const Y = Tt.y0+(j+k*.5)*Tt.res;
        for (let i=0;i+k<nx;i+=k){
          const X = Tt.x0+(i+k*.5)*Tt.res;
          const rr = Math.hypot(X,Y);
          if (rr<rin || rr>=rout) continue;
          if (!(Tt.flag[j*nx+i] & 4)) continue;
          const a=X*ux+Y*uy+X0, b=X*vx+Y*vy+A[j*nx+i]*.001*vz+Y0;
          if (a<L0||a>L1||b<T0||b>T1) continue;
          if (qn>=QMAX) break;
          qk[qn]=(t<<28)|(i<<14)|j; qd[qn++]=X*B.ca+Y*B.sa;
        }
      }
    }
    if (qn<=budget || k>=64) break;
    k <<= 1;
  }

  let lo=1e30,hi=-1e30;
  for (let q=0;q<qn;q++){const v=qd[q]; if(v<lo)lo=v; if(v>hi)hi=v;}
  const NB=1024, s2=(NB-1)/(hi-lo||1);
  qc.fill(0);
  for (let q=0;q<qn;q++) qc[(((qd[q]-lo)*s2)|0)+1]++;
  for (let b=0;b<NB;b++) qc[b+1]+=qc[b];
  for (let q=0;q<qn;q++) qo[qc[((qd[q]-lo)*s2)|0]++]=q;

  const LX=-.42,LY=-.34,LZ=.84;
  const zlo = S.src==='zgnd' ? d.zglo : d.zlo;
  const zsp = (S.src==='zgnd' ? d.zghi-d.zglo : d.zhi-d.zlo) || 1;
  const P = S.paint==='height'?SP : S.paint==='cls'?CL : S.paint==='det'?DT : TV;
  let cur=-1;
  ctx.lineWidth=.6; ctx.strokeStyle=css('--ink');
  for (let o=0;o<qn;o++){
    const key=qk[qo[o]], t=key>>>28, i=(key>>>14)&0x3fff, j=key&0x3fff;
    const Tt=T[t], kk=KS[t], nx=Tt.nx, A=Tt[S.src];
    const p0=j*nx+i, p1=p0+kk, p2=p0+kk*nx+kk, p3=p0+kk*nx;
    const h0=A[p0]*.001,h1=A[p1]*.001,h2=A[p2]*.001,h3=A[p3]*.001;
    const dd=kk*Tt.res, X=Tt.x0+i*Tt.res, Y=Tt.y0+j*Tt.res;
    const ax=X*ux+Y*uy+X0, ay=X*vx+Y*vy+Y0;
    const dux=dd*ux,duy=dd*uy,dvx=dd*vx,dvy=dd*vy;
    const gx=((h1+h2)-(h0+h3))*.5/dd*S.ob.vex, gy=((h3+h2)-(h0+h1))*.5/dd*S.ob.vex;
    const inv=1/Math.sqrt(gx*gx+gy*gy+1);
    let nl=(-gx*LX-gy*LY+LZ)*inv; if(nl<0)nl=0;
    const sh=(.12+.88*nl)*(NSH-1)|0;
    let ci;
    if (S.paint==='height'){ let u=((h0+h1+h2+h3)*.25-zlo)/zsp; ci=u<=0?0:u>=1?255:(u*255)|0; }
    else if (S.paint==='cls') ci=Tt.cls[p0];
    else if (S.paint==='det'){ const v = Tt.det ? Tt.det[p0] : 255; ci = v===255 ? 6 : v; }
    else { const fl=Tt.flag[p0]; ci=(fl&1)?((fl>>1)&1)+1:0; }
    const idx=ci*NSH+sh;
    if (idx!==cur){ cur=idx; ctx.fillStyle=P[idx]; }
    ctx.beginPath();
    ctx.moveTo(ax, ay+h0*vz);
    ctx.lineTo(ax+dux, ay+dvx+h1*vz);
    ctx.lineTo(ax+dux+duy, ay+dvx+dvy+h2*vz);
    ctx.lineTo(ax+duy, ay+dvy+h3*vz);
    ctx.closePath(); ctx.fill();
    if (S.mesh && dd*sc>5.5) ctx.stroke();
  }
  ctx.strokeStyle=css('--mark'); ctx.lineWidth=1.5;
  ctx.beginPath(); ctx.moveTo(X0,Y0); ctx.lineTo(X0, Y0+2.2*vz); ctx.stroke();
  ctx.beginPath(); ctx.arc(X0,Y0,3.5,0,6.2832); ctx.stroke();

  $('tagL').hidden = false; $('tagB').hidden = false;
  $('tagL').textContent = `seq ${d.seq} · frame ${d.frame} · ${d.source==='truth'?'ground truth':'detector'} labels`;
  $('tagB').textContent = `azimuth ${(((S.ob.az*180/Math.PI)%360+360)%360).toFixed(0)}° · `
    + `elevation ${(S.ob.el*180/Math.PI).toFixed(0)}° · height ×${S.ob.vex} · `
    + T.map((t,i)=>(t.res*KS[i]*100).toFixed(0)).join('/') + ' cm mesh';
}
let pend=false;
const redraw=()=>{ if(pend)return; pend=true; requestAnimationFrame(()=>{pend=false;draw();}); };

/* ------------------------------------------------------------------- info */
function info(){
  const d = frameOf(S.frames[S.cur]);
  if (!d){ $('kv').innerHTML='<dt>&mdash;</dt><dd>&mdash;</dd>'; return; }
  const rows = [
    ['points in', fmt(d.npts)],
    ['cells at 5 cm', fmt(d.fine)],
    ['adaptive cells', fmt(d.ncells)],
    ['compression', (d.uniform/d.ncells).toFixed(0)+'×'],
    ['drivable', d.drivable.toFixed(1)+' %'],
    ['tiers 5/10/20/40', d.lvlcount.join(' / ')],
  ];
  if (d.source === 'model')
    rows.push(['clusters', d.clusters], ['cars', d.cars], ['ped / cyclist', d.vru]);
  rows.push(['fetch', d.ms.fetch+' ms'], ['labels', d.ms.label+' ms'],
            ['grid', d.ms.grid+' ms'], ['surface', d.ms.surface+' ms']);
  $('kv').innerHTML = rows.map(r=>`<dt>${r[0]}</dt><dd class="mono">${r[1]}</dd>`).join('');
  legend();
}
function legend(){
  const d = frameOf(S.frames[S.cur]), box = $('legend');
  if (!d){ box.innerHTML=''; return; }
  if (S.paint === 'height'){
    const st=[]; for(let i=0;i<=10;i++) st.push(SPECTRAL[Math.round(i/10*255)]);
    const lo = S.src==='zgnd'?d.zglo:d.zlo, hi = S.src==='zgnd'?d.zghi:d.zhi;
    box.innerHTML = `<div class="bar" style="background:linear-gradient(90deg,${st.join(',')})"></div>`
      + `<div class="ends"><span>${lo.toFixed(2)} m</span>`
      + `<span>${S.src==='ztop'?'top of everything':'terrain only'}</span>`
      + `<span>${hi.toFixed(2)} m</span></div>`;
  } else if (S.paint === 'cls'){
    box.innerHTML = CLASSN.map((n,i)=>
      `<div style="display:flex;align-items:center;gap:7px;font-size:11.5px">
        <i style="width:11px;height:11px;border-radius:2px;background:${CLASSC[i]}"></i>${n}
        <b class="mono" style="margin-left:auto;font-weight:400;color:var(--ink3)">${fmt(d.clscount[i])}</b></div>`).join('')
      + `<div class="ends" style="margin-top:5px"><span>${d.source==='model'?'only car and pedestrian come from the network':'ground truth'}</span></div>`;
  } else if (S.paint === 'det'){
    if (d.source !== 'model'){
      box.innerHTML = `<div class="ends"><span>only available with detector labels &mdash;
        ground truth has no detector to report on</span></div>`;
      return;
    }
    const pc = d.provcount || {};
    const keys = ['ground','background','car','pedestrian','cyclist','unclustered'];
    box.innerHTML = DETN.slice(0,6).map((n,i)=>
      `<div style="display:flex;align-items:center;gap:7px;font-size:11.5px">
        <i style="width:11px;height:11px;border-radius:2px;background:${DETC[i]}"></i>${n}
        <b class="mono" style="margin-left:auto;font-weight:400;color:var(--ink3)">${fmt(pc[keys[i]]||0)}</b></div>`).join('')
      + `<div class="ends" style="margin-top:5px"><span>point counts &middot; the two grey
         entries are what the network never got a say in</span></div>`;
  } else {
    box.innerHTML = [['--ok','drivable'],['--no','blocked'],['--ink3','no returns here']]
      .map(([c,n])=>`<div style="display:flex;align-items:center;gap:7px;font-size:11.5px">
        <i style="width:11px;height:11px;border-radius:2px;background:${css(c)}"></i>${n}</div>`).join('')
      + `<div class="ends" style="margin-top:5px"><span>grey is terrain with no cell, not free space</span></div>`;
  }
}

/* -------------------------------------------------------------- transport */
function show(i){
  if (!S.frames.length) return;
  S.cur = (i + S.frames.length) % S.frames.length;
  const f = S.frames[S.cur];
  $('scrub').value = S.cur;
  $('fnum').textContent = `${S.cur+1} / ${S.frames.length}`;
  [...$('strip').children].forEach((b,n)=>b.classList.toggle('cur', n===S.cur));
  if (frameOf(f)){ info(); redraw(); }
  photo();
}
function tick(t){
  if (!S.playing) return;
  if (t - S.last >= S.spf){
    S.last = t;
    let n = S.cur, tries = 0;
    do { n = (n+1) % S.frames.length; tries++; } while (!frameOf(S.frames[n]) && tries < S.frames.length);
    show(n);
  }
  requestAnimationFrame(tick);
}
$('play').onclick = () => {
  S.playing = !S.playing; $('play').textContent = S.playing?'Pause':'Play';
  S.last = performance.now(); if (S.playing) requestAnimationFrame(tick);
};
$('prev').onclick = () => { S.playing=false; $('play').textContent='Play'; show(S.cur-1); };
$('next').onclick = () => { S.playing=false; $('play').textContent='Play'; show(S.cur+1); };
$('scrub').oninput = e => { S.playing=false; $('play').textContent='Play'; show(+e.target.value); };
$('fitbtn').onclick = () => { fit(); redraw(); };
addEventListener('keydown', e => {
  if (e.target.tagName === 'INPUT' || e.target.tagName === 'SELECT') return;
  if (e.key === 'f' || e.key === 'F'){ fit(); redraw(); }
  else if (e.key === ' '){ e.preventDefault(); $('play').click(); }
  else if (e.key === 'ArrowLeft') $('prev').click();
  else if (e.key === 'ArrowRight') $('next').click();
});

/* ----------------------------------------------------------- view options */
function seg(id, get, set){
  $(id).addEventListener('click', e => {
    const b = e.target.closest('button'); if (!b) return;
    set(b.dataset.v, b);
    [...$(id).children].forEach(x => x.setAttribute('aria-pressed', String(get()===x.dataset.v)));
    legend(); redraw();
  });
}
seg('src', ()=>S.src, v => { S.src=v; setVex(v==='ztop'?2:8); fit(); });
seg('paint', ()=>S.paint, v => S.paint=v);
seg('vex', ()=>String(S.ob.vex), v => setVex(+v));
/* straight down, so all four quadrants of the map are visible at once --
   the photo only ever covers one of them */
let angle = 'tilt';
seg('angle', ()=>angle, v => {
  angle = v;
  S.ob.el = v === 'top' ? 1.48 : 0.40;
  S.ob.az = v === 'top' ? Math.PI : 3.90;
  fit();
});
function setVex(v){
  const f = v/S.ob.vex; S.ob.vex = v;
  S.ob.py0 = H*.5 - (H*.5-S.ob.py0)*f;
  [...$('vex').children].forEach(x=>x.setAttribute('aria-pressed', String(+x.dataset.v===v)));
}
$('toggles').addEventListener('click', e => {
  const b = e.target.closest('button'); if (!b) return;
  S[b.dataset.v] = !S[b.dataset.v];
  b.setAttribute('aria-pressed', String(S[b.dataset.v]));
  if (b.dataset.v === 'camera') photo();
  else if (b.dataset.v === 'proj') overlay();
  else redraw();
});

/* the camera frame for whatever is on screen. it arrives on its own schedule
   -- the map never waits for it, and a frame with no photo just has none. */
function photo(){
  const box = $('cam'), img = $('camimg');
  if (!S.camera || !S.job || S.cur < 0){ box.hidden = true; return; }
  const f = S.frames[S.cur];
  if (img.dataset.f !== f){
    img.dataset.f = f;
    box.classList.add('pending');
    img.onload = () => { if (img.dataset.f === f){ box.classList.remove('pending'); overlay(); } };
    img.onerror = () => { if (img.dataset.f === f) box.hidden = true; };
    img.src = `/api/jobs/${S.job}/image/${f}`;
  }
  box.hidden = false;
  overlay();
}

/* the same laser points, put back through the camera. this is the picture
   that shows the two really are the same scene -- and it only covers the
   82 degrees the lens sees, which is the point being made. */
function overlay(){
  const c = $('camov'), d = frameOf(S.frames[S.cur]);
  const p = d && d.proj;
  const box = $('cam');
  $('projn').textContent = p && S.proj
    ? ` ${fmt(p.n)} of the sweep's points fall inside it.` : '';
  if (!p || !S.proj){ if (c.width) c.getContext('2d').clearRect(0,0,c.width,c.height); return; }
  const r = Math.min(2, devicePixelRatio||1);
  const w = c.clientWidth || 1, h = c.clientHeight || Math.round(w*p.h/p.w);
  c.width = Math.round(w*r); c.height = Math.round(h*r);
  const g = c.getContext('2d');
  g.setTransform(r,0,0,r,0,0); g.clearRect(0,0,w,h);
  if (!d._proj) d._proj = {u:dec(p.u,Uint16Array), v:dec(p.v,Uint16Array), cls:dec(p.cls,Uint8Array)};
  const {u,v,cls} = d._proj, sx = w/p.w, sy = h/p.h;
  const dot = w > 380 ? 1.6 : 1.1;
  let cur = -1;
  for (let i = 0; i < u.length; i++){
    if (cls[i] !== cur){ cur = cls[i]; g.fillStyle = CLASSC[cur]; }
    g.fillRect(u[i]*sx, v[i]*sy, dot, dot);
  }
}

/* -------------------------------------------------------------- camera */
cv.addEventListener('pointerdown', e => {
  S.drag = {x:e.clientX, y:e.clientY, az:S.ob.az, el:S.ob.el, ox:S.ob.px0, oy:S.ob.py0};
  cv.setPointerCapture(e.pointerId); cv.style.cursor='grabbing';
});
cv.addEventListener('pointerup', ()=>{ S.drag=null; cv.style.cursor='grab'; redraw(); });
cv.addEventListener('pointermove', e => {
  if (!S.drag) return;
  const dx=e.clientX-S.drag.x, dy=e.clientY-S.drag.y;
  if (e.shiftKey){ S.ob.px0=S.drag.ox+dx; S.ob.py0=S.drag.oy+dy; }
  else { S.ob.az=S.drag.az-dx*.006;
         S.ob.el=Math.max(.06, Math.min(1.45, S.drag.el+dy*.004)); }
  S.touched = true;
  redraw();
});
cv.addEventListener('wheel', e => {
  e.preventDefault();
  const r=cv.getBoundingClientRect(), mx=e.clientX-r.left, my=e.clientY-r.top;
  const s2=Math.min(900, Math.max(.4, S.ob.S*Math.pow(.999, e.deltaY)));
  S.ob.px0 = mx-(mx-S.ob.px0)*(s2/S.ob.S);
  S.ob.py0 = my-(my-S.ob.py0)*(s2/S.ob.S);
  S.ob.S = s2; S.touched = true; redraw();
}, {passive:false});
function resize(){
  const r = cv.parentElement.getBoundingClientRect();
  dpr = Math.min(2, devicePixelRatio||1);
  W = r.width; H = r.height;
  cv.width = Math.max(1, Math.round(W*dpr)); cv.height = Math.max(1, Math.round(H*dpr));
  if (!S.touched) fit();
  redraw();
}
new ResizeObserver(resize).observe(cv.parentElement);
matchMedia('(prefers-color-scheme: dark)').addEventListener('change', ()=>{ SP=null; legend(); redraw(); });

/* ------------------------------------------------------------------- job */
function setState(s, cls){ $('state').textContent = s; $('dot').className = 'dot '+(cls||''); }

seg('mode', ()=>document.querySelector('#mode [aria-pressed=true]').dataset.v, v=>{
  const rnd = v === 'random';
  $('start').disabled = rnd; $('stride').disabled = rnd; $('seed').disabled = !rnd;
  $('hint').textContent = rnd
    ? 'Random frames are scattered across the whole sequence — unrelated scenes, good for coverage.'
    : 'Sequential frames are consecutive sweeps — real motion. Nothing is precomputed: each frame is pulled, labelled, converted and reduced when you ask for it.';
});

async function run(){
  if (S.es) S.es.close();
  S.data.clear(); S.frames=[]; S.cur=-1; S.playing=false;
  $('play').textContent='Play'; $('err').textContent=''; $('empty').style.display='';
  $('tagL').hidden = $('tagB').hidden = true;
  $('cam').hidden = true; $('camimg').dataset.f = '';
  const spec = {
    seq: $('seq').value,
    mode: document.querySelector('#mode [aria-pressed=true]').dataset.v,
    start: +$('start').value, count: +$('count').value,
    stride: +$('stride').value, source: $('source').value, seed: +$('seed').value,
    camera: S.camera,
  };
  $('go').disabled = true; $('stop').hidden = false;
  setState('starting', 'run');
  let r;
  try {
    r = await (await fetch('/api/jobs', {method:'POST',
      headers:{'Content-Type':'application/json'}, body:JSON.stringify(spec)})).json();
  } catch(e){ setState('failed','err'); $('err').textContent = String(e);
              $('go').disabled=false; $('stop').hidden=true; return; }
  if (r.detail){ setState('rejected','err'); $('err').textContent = r.detail;
                 $('go').disabled=false; $('stop').hidden=true; return; }

  S.job = r.id; S.frames = r.frames;
  $('scrub').max = S.frames.length-1;
  $('transport').hidden = false;
  $('strip').innerHTML = S.frames.map((f,i)=>
    `<div class="cellbox" data-i="${i}" title="frame ${f}">${i+1}</div>`).join('');
  $('strip').onclick = e => {
    const b = e.target.closest('.cellbox');
    if (b && b.classList.contains('ready')){ S.playing=false; $('play').textContent='Play'; show(+b.dataset.i); }
  };
  $('strip').children[0]?.classList.add('busy');
  let ready = 0;
  setState(`running 0/${S.frames.length}`, 'run');

  let finished = false;
  S.es = new EventSource(`/api/jobs/${r.id}/events`);
  S.es.onmessage = async ev => {
    const m = JSON.parse(ev.data);
    if (m.type === 'frame'){
      const d = await (await fetch(`/api/jobs/${r.id}/frame/${m.frame}`)).json();
      S.data.set(m.frame, d);
      const box = $('strip').children[m.index];
      box.classList.remove('busy'); box.classList.add('ready');
      box.title = `frame ${m.frame} · ${fmt(d.ncells)} cells · ${d.drivable}% drivable`
        + (m.cached ? ' · cached' : '');
      $('strip').children[m.index+1]?.classList.add('busy');
      ready++;
      if (!finished) setState(`running ${ready}/${S.frames.length}`, 'run');
      $('prog').textContent = `${ready} of ${S.frames.length} ready`
        + `  ·  fetch ${m.ms.fetch} ms, labels ${m.ms.label} ms, grid ${m.ms.grid} ms`;
      if (S.cur < 0){ show(m.index); fit(); redraw(); }
    } else if (m.type === 'error'){
      const box = $('strip').children[m.index];
      box.classList.remove('busy'); box.classList.add('err'); box.title = m.error;
      $('err').textContent = m.error;
      $('strip').children[m.index+1]?.classList.add('busy');
    } else if (m.type === 'end'){
      finished = true;
      S.es.close(); S.es = null;
      setState(m.state === 'done' ? `${S.frames.length} frames ready` : m.state,
               m.errors ? 'err' : 'done');
      $('go').disabled = false; $('stop').hidden = true;
      [...$('strip').children].forEach(b=>b.classList.remove('busy'));
    }
  };
  S.es.onerror = () => { setState('stream lost','err'); $('go').disabled=false; $('stop').hidden=true; };
}
$('go').onclick = run;
$('stop').onclick = async () => { if (S.job) await fetch(`/api/jobs/${S.job}/cancel`, {method:'POST'}); };

(async function init(){
  const seqs = await (await fetch('/api/sequences')).json();
  $('seq').innerHTML = seqs.map(s =>
    `<option value="${s.seq}">${s.seq} — ${fmt(s.frames)} frames</option>`).join('');
  resize();

  /* the whole run is addressable: ?seq=00&mode=sequential&count=8&auto=1 */
  const q = new URLSearchParams(location.search);
  for (const k of ['seq','source','start','count','stride','seed'])
    if (q.has(k)) $(k).value = q.get(k);
  if (q.has('mode')){
    [...$('mode').children].forEach(b =>
      b.setAttribute('aria-pressed', String(b.dataset.v === q.get('mode'))));
    const rnd = q.get('mode') === 'random';
    $('start').disabled = rnd; $('stride').disabled = rnd; $('seed').disabled = !rnd;
  }
  for (const [id, k] of [['src','src'], ['paint','paint']])
    if (q.has(k)){
      S[k] = q.get(k);
      [...$(id).children].forEach(b => b.setAttribute('aria-pressed', String(b.dataset.v === q.get(k))));
    }
  if (q.has('vex')) setVex(+q.get('vex'));
  if (q.get('auto') === '1') run();
})();
