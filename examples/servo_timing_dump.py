#!/usr/bin/env python3
"""Export a stopped session's tmpfs servo timing rings; not part of the UDP/API protocol."""
import argparse
import csv
import json
import struct
from pathlib import Path

UDP_FIELDS = ['message_id', 'receive_ns', 'locked_ns', 'ready_ns', 'published_ns',
              'disposition', 'reserved0', 'reserved1']
CONTROL_FIELDS = ['cycle', 'entry_ns', 'locked_ns', 'consumed_ns', 'consumed_id',
                  'reference_id', 'applied_id', 'completed_ns', 'result', 'reserved']


def export(source, destination):
    # 活动会话不保证跨生产者一致，只在会话停止后读取；计数变化则拒绝本次导出。
    with source.open('rb') as stream:
        header = stream.read(256)
        if len(header) != 256 or header[:8] != b'RCSTIME1':
            raise ValueError('Not a servo timing v1 capture')
        version, capacity, udp_size, control_size, mono, real = struct.unpack_from('<6Q', header, 8)
        if version != 1 or (udp_size, control_size) != (64, 80) or not 0 < capacity <= 65536:
            raise ValueError('Unsupported trace layout (little-endian v1 required)')
        session = header[56:96].split(b'\0')[0].decode('ascii')
        counts = (struct.unpack_from('<Q', header, 128)[0], struct.unpack_from('<Q', header, 192)[0])
        if source.stat().st_size != 256 + capacity * (udp_size + control_size):
            raise ValueError('Truncated trace')
        records = []
        for count, size, offset, fmt in [
                (counts[0], udp_size, 256, '<8Q'),
                (counts[1], control_size, 256 + udp_size * capacity, '<8QqQ')]:
            stream.seek(offset)
            block = stream.read(size * capacity)
            records.append([struct.unpack_from(fmt, block, (index % capacity) * size)
                            for index in range(max(0, count - capacity), count)])
        stream.seek(0)
        after = stream.read(256)
        after_counts = (struct.unpack_from('<Q', after, 128)[0], struct.unpack_from('<Q', after, 192)[0])
        if after_counts != counts:
            raise ValueError('Capture is still active; stop the servo session before exporting')
    destination.mkdir(parents=True, exist_ok=False)
    for name, fields, rows in [('udp.csv', UDP_FIELDS, records[0]),
                               ('control.csv', CONTROL_FIELDS, records[1])]:
        with (destination / name).open('w', newline='') as target:
            writer = csv.writer(target)
            writer.writerow(fields)
            writer.writerows(rows)
    metadata = dict(version=version, capacity=capacity, session_id=session,
                    clock='controller CLOCK_MONOTONIC, nanoseconds',
                    created_mono_ns=mono, created_real_ns=real,
                    udp_total=counts[0], control_total=counts[1],
                    udp_overwritten=max(0, counts[0] - capacity),
                    control_overwritten=max(0, counts[1] - capacity),
                    source=str(source.resolve()))
    (destination / 'metadata.json').write_text(json.dumps(metadata, indent=2) + '\n')
    return metadata


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('capture', type=Path)
    parser.add_argument('output', type=Path, help='New output directory')
    args = parser.parse_args()
    print(json.dumps(export(args.capture, args.output), indent=2))
