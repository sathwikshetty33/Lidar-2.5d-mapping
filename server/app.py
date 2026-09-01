"""
the pipeline as a service.

a job is a list of frames. a worker thread walks it, running fetch -> label ->
grid -> surface for each one and pushing an event when it lands. the browser
subscribes to those events and draws frames as they arrive, so a 20-frame
sequential run starts playing before it has finished downloading.
"""

from __future__ import annotations

import asyncio, json, queue, threading, traceback, uuid
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

from fastapi import FastAPI, HTTPException
from fastapi.responses import FileResponse, StreamingResponse
from fastapi.staticfiles import StaticFiles
from pydantic import BaseModel

import pipeline as P

ROOT = Path(__file__).resolve().parent.parent
app = FastAPI(title='Adaptive 2.5D pipeline')

JOBS: dict[str, dict] = {}
_lock = threading.Lock()


class JobSpec(BaseModel):
    seq: str = '00'
    mode: str = 'sequential'      # sequential | random
    start: int = 0
    count: int = 8
    stride: int = 1
    source: str = 'model'         # model | truth
    seed: int = 0
    camera: bool = True           # also pull the matching camera frame
    detail: int = 4               # surface base = 5 cm x this (4=20cm, 2=10cm, 1=5cm)


def _run(job_id: str):
    job = JOBS[job_id]
    # start every download at once, three at a time, while the main loop
    # labels and converts whatever has landed. fetching dominates a cold run
    # (~38 s a frame) and it is pure I/O, so overlapping it is nearly free.
    want_truth = job['source'] == 'truth'
    # photos are pulled on a separate, lower-priority lane so a slow image
    # never delays the map it belongs to
    if job['camera']:
        for f in job['frames']:
            job['cam_pool'].submit(P.prefetch_image, job['seq'], f)
    pool = ThreadPoolExecutor(max_workers=3, thread_name_prefix='fetch')
    futs = {f: pool.submit(P.prefetch, job['seq'], f, want_truth)
            for f in job['frames']}
    for i, fid in enumerate(job['frames']):
        if job['cancel']:
            break
        try:
            futs[fid].result()                 # its download is done (or failed)
            out = P.build(job['seq'], fid, job['source'], job['detail'])
            with _lock:
                job['done'][fid] = out
                job['order'].append(fid)
            ev = dict(type='frame', index=i, frame=fid, cached=out['cached'],
                      npts=out['npts'], ncells=out['ncells'],
                      drivable=out['drivable'], ms=out['ms'])
        except Exception as e:                       # keep going; report it
            traceback.print_exc()
            with _lock:
                job['errors'].append({'frame': fid, 'error': str(e)})
            ev = dict(type='error', index=i, frame=fid, error=str(e))
        job['events'].put(ev)
    pool.shutdown(wait=False, cancel_futures=True)
    job['cam_pool'].shutdown(wait=False, cancel_futures=True)
    job['state'] = 'cancelled' if job['cancel'] else 'done'
    job['events'].put(dict(type='end', state=job['state'],
                           errors=len(job['errors'])))


@app.get('/api/sequences')
def sequences():
    return [{'seq': s, 'frames': n,
             'labels': s in ('00','01','02','03','04','05','06','07','08','09','10')}
            for s, n in sorted(P.SEQ_LEN.items())]


@app.post('/api/jobs')
def create(spec: JobSpec):
    if spec.count < 1 or spec.count > 60:
        raise HTTPException(400, 'count must be 1..60')
    if spec.seq not in P.SEQ_LEN:
        raise HTTPException(400, f'unknown sequence {spec.seq}')
    frames = P.frame_ids(spec.seq, spec.mode, spec.start, spec.count,
                         max(1, spec.stride), spec.seed)
    if not frames:
        raise HTTPException(400, 'that range contains no frames')
    jid = uuid.uuid4().hex[:12]
    JOBS[jid] = dict(id=jid, seq=spec.seq, source=spec.source, frames=frames,
                     camera=spec.camera, detail=max(1, min(4, spec.detail)),
                     done={}, order=[], errors=[],
                     events=queue.Queue(), state='running', cancel=False,
                     cam_pool=ThreadPoolExecutor(max_workers=2, thread_name_prefix='cam'),
                     spec=spec.model_dump())
    threading.Thread(target=_run, args=(jid,), daemon=True).start()
    return {'id': jid, 'frames': frames, 'state': 'running'}


@app.get('/api/jobs/{jid}')
def status(jid: str):
    j = JOBS.get(jid)
    if not j:
        raise HTTPException(404, 'no such job')
    return {'id': jid, 'state': j['state'], 'total': len(j['frames']),
            'ready': list(j['order']), 'errors': j['errors'],
            'spec': j['spec']}


@app.post('/api/jobs/{jid}/cancel')
def cancel(jid: str):
    j = JOBS.get(jid)
    if not j:
        raise HTTPException(404, 'no such job')
    j['cancel'] = True
    return {'ok': True}


@app.get('/api/jobs/{jid}/events')
async def events(jid: str):
    j = JOBS.get(jid)
    if not j:
        raise HTTPException(404, 'no such job')

    async def gen():
        loop = asyncio.get_event_loop()
        while True:
            try:
                ev = await loop.run_in_executor(None, j['events'].get, True, 30)
            except queue.Empty:
                yield ': keepalive\n\n'
                continue
            yield f'data: {json.dumps(ev)}\n\n'
            if ev.get('type') == 'end':
                return

    return StreamingResponse(gen(), media_type='text/event-stream',
                             headers={'Cache-Control': 'no-cache',
                                      'X-Accel-Buffering': 'no'})


@app.get('/api/jobs/{jid}/frame/{fid}')
def frame(jid: str, fid: str):
    j = JOBS.get(jid)
    if not j:
        raise HTTPException(404, 'no such job')
    out = j['done'].get(fid)
    if out is None:
        raise HTTPException(404, 'frame not ready')
    return out


@app.get('/api/jobs/{jid}/image/{fid}')
def image(jid: str, fid: str):
    j = JOBS.get(jid)
    if not j:
        raise HTTPException(404, 'no such job')
    try:
        p = P.image(j['seq'], fid)
    except Exception as e:
        raise HTTPException(404, f'no camera frame: {e}')
    return FileResponse(p, media_type='image/png',
                        headers={'Cache-Control': 'public, max-age=86400'})


@app.get('/api/health')
def health():
    return {'ok': True, 'cached_frames': len(list(P.OUT.glob('*.json'))),
            'cached_raw': len(list(P.RAW.glob('*.bin')))}


app.mount('/', StaticFiles(directory=ROOT / 'web', html=True), name='web')
