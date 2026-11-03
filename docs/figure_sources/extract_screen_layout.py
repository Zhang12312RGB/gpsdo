"""Read active page records from the user's HMI without modifying the HMI."""
from pathlib import Path
import hashlib
import json
import struct

B = Path(__file__).resolve().parent
SOURCE = Path(r'D:\FPGAdesigner\TJC\TJC.HMI')
raw = SOURCE.read_bytes()


def property_value(obj, name):
    marker = name.encode('ascii').ljust(16, b'\0')
    idx = obj.find(marker)
    if idx < 4:
        return None
    size = struct.unpack_from('<I', obj, idx - 4)[0]
    assert 16 <= size <= len(obj) - idx
    data = obj[idx + 16:idx + size]
    if name in ('objname', 'txt'):
        return data.decode('gb18030')
    return int.from_bytes(data, 'little')


pages = []
entry_count = struct.unpack_from('<I', raw)[0]
for i in range(entry_count):
    entry = 4 + 28 * i
    name = raw[entry:entry + 16].split(b'\0')[0]
    offset, length, flags = struct.unpack_from('<III', raw, entry + 16)
    # Deleted archive entries have a cleared first filename byte.
    if name not in (b'0.pa', b'1.pa'):
        continue
    assert offset + length <= len(raw)
    page = raw[offset:offset + length]
    size, header, count = struct.unpack_from('<III', page, 4)
    assert size == length and header == 56
    objects = []
    for j in range(count):
        start, obj_size, unused = struct.unpack_from('<III', page, header + 12 * j)
        assert header + start + obj_size <= length
        obj = page[header + start:header + start + obj_size]
        item = {key: property_value(obj, key)
                for key in ('objname', 'x', 'y', 'w', 'h', 'vscope', 'txt')}
        item['object_offset'] = offset + header + start
        objects.append(item)
    pages.append({'entry': name.decode(), 'offset': offset,
                  'bytes': length, 'objects': objects})

expected = [
    ['page0', 'gpstxt', 'gpfrq', 'gpphs', 'gpt', 'gpslw', 'gppps'],
    ['page1', 'gpsl', 'gpsn', 'gphd', 'gpcn', 'gpsw', 'gpsj', 'gpst', 'gpsd'],
]
assert [[o['objname'] for o in p['objects']] for p in pages] == expected
for page in pages:
    assert (page['objects'][0]['w'], page['objects'][0]['h']) == (128, 160)
    assert [o['y'] for o in page['objects'][1:]] == list(range(0, 20 * (len(page['objects']) - 1), 20))

evidence = {'source': str(SOURCE), 'sha256': hashlib.sha256(raw).hexdigest(),
            'method': 'Read active named archive records and serialized object properties; no HMI write or screen flash.',
            'on_device_flashed_layout_verified': False, 'pages': pages}
(B / 'screen_layout_evidence.json').write_text(
    json.dumps(evidence, ensure_ascii=False, indent=2), encoding='utf-8')
print(json.dumps({'pages': len(pages), 'controls': [len(p['objects']) - 1 for p in pages],
                  'page0': expected[0][1:], 'page1': expected[1][1:]}, ensure_ascii=False))
