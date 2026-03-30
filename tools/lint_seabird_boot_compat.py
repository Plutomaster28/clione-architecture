#!/usr/bin/env python3
"""Report boot-assembly mnemonics unsupported by the current Clione64 bring-up assembler profile."""

from __future__ import annotations

import argparse
import json
import pathlib
import re
from collections import Counter

COMMENT_RE = re.compile(r"(;|//).*$")
LABEL_RE = re.compile(r"^([A-Za-z_.][A-Za-z0-9_.]*):$")


def load_spec(spec_path: pathlib.Path) -> dict:
  spec = json.loads(spec_path.read_text(encoding="utf-8"))
  base_ref = spec.get("base_spec")
  if not base_ref:
    return spec

  base_path = (spec_path.parent / base_ref).resolve()
  base = load_spec(base_path)
  merged = dict(base)
  merged["instructions"] = dict(base.get("instructions", {}))
  merged["instructions"].update(spec.get("instructions", {}))
  merged["aliases"] = dict(base.get("aliases", {}))
  merged["aliases"].update(spec.get("aliases", {}))
  for k, v in spec.items():
    if k in ("instructions", "aliases"):
      continue
    merged[k] = v
  return merged


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("asm", help="Assembly file to check")
    ap.add_argument("--spec", default="tools/isa/clione64_isa.json", help="ISA JSON spec")
    args = ap.parse_args()

    spec = load_spec(pathlib.Path(args.spec))
    supported = set(spec.get("instructions", {}).keys())
    aliases = set(spec.get("aliases", {}).keys())
    supported |= aliases
    if spec.get("profile_mode") == "seabird_boot":
      supported |= set(spec.get("seabird_boot_surface", []))
      supported |= set(spec.get("seabird_boot_supported_mnemonics", []))

    src = pathlib.Path(args.asm).read_text(encoding="utf-8").splitlines()
    unsupported: Counter[str] = Counter()

    for line in src:
      text = COMMENT_RE.sub("", line).strip()
      if not text:
        continue
      if LABEL_RE.match(text):
        continue
      if text.startswith("."):
        continue
      mnemonic = text.split(None, 1)[0].upper()
      if mnemonic not in supported:
        unsupported[mnemonic] += 1

    print(f"Checked: {args.asm}")
    if not unsupported:
      print("All mnemonics are supported by current profile.")
      return 0

    print("Unsupported mnemonics (count):")
    for mn, cnt in unsupported.most_common():
      print(f"  {mn}: {cnt}")
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
