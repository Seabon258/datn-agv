#!/usr/bin/env python3
"""Them goi y in vao PDF (incremental update, khong viet lai noi dung):
   /Duplex /DuplexFlipLongEdge  -> may in tu chon in 2 mat lat canh dai
   /PrintScaling /None          -> khong co gian (in 100%)
    python3 pdf_duplex.py file.pdf
"""
import re
import sys


def add_hints(path):
    data = open(path, 'rb').read()
    root = int(re.findall(rb'/Root\s+(\d+)\s+0\s+R', data)[-1])
    size = int(re.findall(rb'/Size\s+(\d+)', data)[-1])
    prev = int(re.findall(rb'startxref\s+(\d+)', data)[-1])
    m = list(re.finditer(rb'(?<!\d)%d\s+0\s+obj\s*<<(.*?)>>\s*endobj' % root, data, re.S))[-1]
    body = re.sub(rb'/ViewerPreferences\s*<<.*?>>', b'', m.group(1), flags=re.S)
    obj = b'%d 0 obj\n<<%s /ViewerPreferences << /Duplex /DuplexFlipLongEdge /PrintScaling /None >> >>\nendobj\n' % (root, body)
    if not data.endswith(b'\n'):
        data += b'\n'
    off = len(data)
    xref = b'xref\n%d 1\n%010d 00000 n \n' % (root, off)
    trailer = b'trailer\n<< /Size %d /Root %d 0 R /Prev %d >>\nstartxref\n%d\n%%%%EOF\n' % (size, root, prev, off + len(obj))
    open(path, 'wb').write(data + obj + xref + trailer)


if __name__ == '__main__':
    for p in sys.argv[1:]:
        add_hints(p)
        print('da them goi y in 2 mat (lat canh dai) + 100%:', p)
