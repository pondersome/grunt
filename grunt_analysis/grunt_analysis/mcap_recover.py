"""Recover a truncated mcap bag (recorder killed before the footer).

`ros2 bag record` killed mid-write — e.g. by a hard `kill -9`, or a
crash — leaves an mcap whose summary/footer is missing or corrupt. The
indexed reader then fails outright (`RecordLengthLimitExceeded` or
similar) even though the message data before the truncation is fine.

This rescans the file sequentially with StreamReader, which tolerates
a bad tail, and rewrites a clean, finalised mcap. Everything up to the
truncation is preserved; only the final partial chunk is lost.

Run:
    python -m grunt_analysis.mcap_recover <bag_dir|file.mcap> [--out DIR]

then point the analysis tools at the `<bag>_recovered` directory.
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

from mcap.stream_reader import StreamReader
from mcap.writer import Writer
from mcap.records import Schema, Channel, Message

from .bag import discover_mcap


def recover(src: Path, dst: Path) -> dict:
    """Rewrite mcap `src` into a clean, finalised mcap at `dst`.

    Returns a summary dict: n_msg, per_topic counts, and `stopped` (the
    exception string where the readable data ended, or None if `src`
    was intact). Safe to run on a non-truncated bag — it just copies.
    """
    schema_map: dict[int, int] = {}     # original id -> writer id
    channel_map: dict[int, int] = {}
    topic_of: dict[int, str] = {}
    per_topic: dict[int, int] = {}
    n_msg = 0
    stopped = None

    dst.parent.mkdir(parents=True, exist_ok=True)
    with open(dst, "wb") as out:
        w = Writer(out)
        w.start(profile="ros2", library="grunt_analysis.mcap_recover")
        try:
            with open(src, "rb") as f:
                for rec in StreamReader(f, emit_chunks=False).records:
                    if isinstance(rec, Schema):
                        if rec.id not in schema_map:
                            schema_map[rec.id] = w.register_schema(
                                rec.name, rec.encoding, rec.data)
                    elif isinstance(rec, Channel):
                        if rec.id not in channel_map:
                            channel_map[rec.id] = w.register_channel(
                                rec.topic, rec.message_encoding,
                                schema_map.get(rec.schema_id, 0),
                                rec.metadata)
                            topic_of[rec.id] = rec.topic
                    elif isinstance(rec, Message):
                        cid = channel_map.get(rec.channel_id)
                        if cid is not None:
                            w.add_message(cid, rec.log_time, rec.data,
                                          rec.publish_time, rec.sequence)
                            n_msg += 1
                            per_topic[rec.channel_id] = \
                                per_topic.get(rec.channel_id, 0) + 1
        except Exception as e:      # truncated tail — keep what we have
            stopped = f"{type(e).__name__}: {e}"
        w.finish()

    return {
        "n_msg": n_msg,
        "stopped": stopped,
        "dst": dst,
        "per_topic": {topic_of.get(c, str(c)): n
                      for c, n in sorted(per_topic.items())},
    }


def main(argv=None) -> int:
    p = argparse.ArgumentParser(
        description="Recover a truncated mcap bag")
    p.add_argument("bag", help="bag directory or .mcap file")
    p.add_argument("--out", help="output directory "
                   "(default: <bag>_recovered, next to the input)")
    args = p.parse_args(argv)

    src = discover_mcap(args.bag)
    inp = Path(args.bag)
    bag_dir = inp if inp.is_dir() else inp.parent
    out_dir = (Path(args.out) if args.out
               else bag_dir.parent / f"{bag_dir.name}_recovered")
    dst = out_dir / f"{src.stem}_recovered.mcap"

    print(f"recovering {src}", file=sys.stderr)
    r = recover(src, dst)
    print(f"recovered {r['n_msg']} messages:")
    for topic, n in r["per_topic"].items():
        print(f"  {topic:34s} {n}")
    if r["stopped"]:
        print(f"stream ended at: {r['stopped']}  "
              f"(truncated tail dropped)")
    else:
        print("source was intact — clean copy")
    print(f"wrote {dst}")
    print(f"analyse with the recovered directory: {out_dir}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
