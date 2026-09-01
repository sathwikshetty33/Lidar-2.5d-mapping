"""
pull a few semantickitti frames without downloading the 80 gb archive.

zipfile only needs a seekable file object, so a tiny http range reader is
enough to list the central directory and inflate single members. we pull
~2 mb of velodyne and ~0.5 mb of labels per frame instead of the whole set.
"""

import io, os, sys, zipfile, urllib.request

VEL = 'https://s3.eu-central-1.amazonaws.com/avg-kitti/data_odometry_velodyne.zip'
LAB = 'http://www.semantic-kitti.org/assets/data_odometry_labels.zip'
OUT = 'kitti'


class httpfile(io.RawIOBase):
    """seekable read-only view of a remote file, backed by range requests."""

    def __init__(self, url, blk=1 << 18):
        self.url, self.blk, self.pos, self.cache = url, blk, 0, {}
        r = urllib.request.Request(url, method='HEAD')
        with urllib.request.urlopen(r, timeout=60) as h:
            self.size = int(h.headers['Content-Length'])

    def seekable(self):
        return True

    def readable(self):
        return True

    def tell(self):
        return self.pos

    def seek(self, off, whence=0):
        self.pos = off if whence == 0 else self.pos + off if whence == 1 else self.size + off
        return self.pos

    def _block(self, b):
        c = self.cache.get(b)
        if c is None:
            a, z = b * self.blk, min((b + 1) * self.blk, self.size) - 1
            r = urllib.request.Request(self.url, headers={'Range': f'bytes={a}-{z}'})
            with urllib.request.urlopen(r, timeout=120) as h:
                c = h.read()
            # evict BEFORE inserting, and never the block just fetched: the old
            # order dropped the 32 lowest keys after inserting, so a low-numbered
            # block evicted itself and the next line raised KeyError
            if len(self.cache) > 96:
                for k in sorted(self.cache)[:32]:
                    if k != b:
                        self.cache.pop(k, None)
            self.cache[b] = c
        return c

    def read(self, n=-1):
        if n < 0:
            n = self.size - self.pos
        n = min(n, self.size - self.pos)
        out = bytearray()
        while n > 0:
            b, o = divmod(self.pos, self.blk)
            chunk = self._block(b)[o:o + n]
            if not chunk:
                break
            out += chunk
            self.pos += len(chunk)
            n -= len(chunk)
        return bytes(out)


def grab(url, members, dest):
    zf = zipfile.ZipFile(httpfile(url))
    names = set(zf.namelist())
    got = []
    for m in members:
        if m not in names:
            print('  MISSING', m)
            continue
        p = os.path.join(dest, os.path.basename(m))
        data = zf.read(m)          # read fully BEFORE opening the file, or a
        with open(p, 'wb') as f:   # failed read leaves a 0-byte file behind
            f.write(data)
        print(f'  {m}  ->  {os.path.getsize(p):,} bytes')
        got.append(p)
    return got


if __name__ == '__main__':
    seq = sys.argv[1] if len(sys.argv) > 1 else '00'
    frames = sys.argv[2:] or ['000000']
    os.makedirs(OUT, exist_ok=True)
    print('velodyne:')
    grab(VEL, [f'dataset/sequences/{seq}/velodyne/{f}.bin' for f in frames], OUT)
    print('labels:')
    grab(LAB, [f'dataset/sequences/{seq}/labels/{f}.label' for f in frames], OUT)
