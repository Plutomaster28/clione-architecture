#!/usr/bin/env python3
"""Minimal spec-driven assembler for Clione64 RTL bring-up.

This is intentionally strict and small:
- Consumes instruction encoding definitions from tools/isa/clione64_isa.json.
- Supports enough syntax for directed simulation tests.
- Emits 32-bit instruction words as hex/bin/sv.
"""

from __future__ import annotations

import argparse
import ast
import json
import pathlib
import re
import struct
import sys
from dataclasses import dataclass
from typing import Dict, List, Tuple


REG_RE = re.compile(r"^[RrXx](\d+)$")
LABEL_RE = re.compile(r"^([A-Za-z_.][A-Za-z0-9_.]*):$")
COMMENT_RE = re.compile(r"(;|//).*$")


@dataclass
class AsmLine:
    line_no: int
    text: str
    pc: int


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
    merged["register_aliases"] = dict(base.get("register_aliases", {}))
    merged["register_aliases"].update(spec.get("register_aliases", {}))
    for k, v in spec.items():
        if k in ("instructions", "aliases", "register_aliases"):
            continue
        merged[k] = v
    return merged


def _eval_expr(node: ast.AST, symbols: Dict[str, int]) -> int:
    if isinstance(node, ast.Constant):
        if isinstance(node.value, int):
            return int(node.value)
        raise ValueError("Only integer constants are allowed")
    if isinstance(node, ast.Name):
        if node.id not in symbols:
            raise ValueError(f"Unknown symbol '{node.id}'")
        return int(symbols[node.id])
    if isinstance(node, ast.UnaryOp):
        v = _eval_expr(node.operand, symbols)
        if isinstance(node.op, ast.USub):
            return -v
        if isinstance(node.op, ast.UAdd):
            return +v
        if isinstance(node.op, ast.Invert):
            return ~v
        raise ValueError("Unsupported unary operator")
    if isinstance(node, ast.BinOp):
        l = _eval_expr(node.left, symbols)
        r = _eval_expr(node.right, symbols)
        if isinstance(node.op, ast.Add):
            return l + r
        if isinstance(node.op, ast.Sub):
            return l - r
        if isinstance(node.op, ast.Mult):
            return l * r
        if isinstance(node.op, ast.FloorDiv):
            return l // r
        if isinstance(node.op, ast.BitOr):
            return l | r
        if isinstance(node.op, ast.BitAnd):
            return l & r
        if isinstance(node.op, ast.BitXor):
            return l ^ r
        if isinstance(node.op, ast.LShift):
            return l << r
        if isinstance(node.op, ast.RShift):
            return l >> r
        raise ValueError("Unsupported binary operator")
    raise ValueError("Unsupported expression")


def eval_expr(expr: str, symbols: Dict[str, int]) -> int:
    tree = ast.parse(expr, mode="eval")
    return _eval_expr(tree.body, symbols)


def parse_int(token: str, symbols: Dict[str, int] | None = None) -> int:
    token = token.strip()
    if token.startswith("#"):
        token = token[1:]
    if token.startswith("(") and token.endswith(")"):
        token = token[1:-1].strip()
    if symbols and token in symbols:
        return symbols[token]
    if token.startswith("'") and token.endswith("'") and len(token) >= 3:
        body = token[1:-1]
        if body == r"\n":
            return ord("\n")
        if len(body) == 1:
            return ord(body)
    if symbols and re.search(r"[()+\-*/|&^~<>]", token):
        return eval_expr(token, symbols)
    return int(token, 0)


def parse_reg(token: str, spec: dict) -> int:
    t = token.strip()
    aliases = spec.get("register_aliases", {})
    t = aliases.get(t.upper(), t)
    m = REG_RE.match(t)
    if not m:
        raise ValueError(f"Invalid register '{token}'")
    reg = int(m.group(1), 10)
    if not 0 <= reg <= 31:
        raise ValueError(f"Register out of range: {token}")
    return reg


def split_operands(operand_text: str) -> List[str]:
    out: List[str] = []
    cur: List[str] = []
    depth = 0
    for ch in operand_text:
        if ch == "[":
            depth += 1
        elif ch == "]":
            depth = max(depth - 1, 0)
        if ch == "," and depth == 0:
            out.append("".join(cur).strip())
            cur = []
        else:
            cur.append(ch)
    if cur:
        out.append("".join(cur).strip())
    return [x for x in out if x]


def parse_mem_operand(text: str, symbols: Dict[str, int], spec: dict) -> Tuple[int, int]:
    """Parse [R2+16], [R2-8], [R2], [0x1000] into (base, offset)."""
    t = text.strip()
    if not (t.startswith("[") and t.endswith("]")):
        raise ValueError(f"Expected memory operand [..], got '{text}'")
    inner = t[1:-1].replace(" ", "")
    if not inner:
        raise ValueError("Empty memory operand")

    for op in ["+", "-"]:
        if op in inner and not inner.startswith(op):
            base_txt, off_txt = inner.split(op, 1)
            try:
                base = parse_reg(base_txt, spec)
                off = parse_int(off_txt, symbols)
                if op == "-":
                    off = -off
                return base, off
            except ValueError:
                # Treat symbolic arithmetic like [UART_BASE + 5] as absolute addr.
                return 0, parse_int(inner, symbols)

    if REG_RE.match(inner):
        return parse_reg(inner, spec), 0

    aliases = spec.get("register_aliases", {})
    if inner.upper() in aliases:
        return parse_reg(inner, spec), 0

    # Absolute addressing maps to x0 + imm for this profile.
    return 0, parse_int(inner, symbols)


def encode_signed(value: int, bits: int) -> int:
    lo = -(1 << (bits - 1))
    hi = (1 << (bits - 1)) - 1
    if value < lo or value > hi:
        raise ValueError(f"Immediate {value} out of range for {bits}-bit signed")
    return value & ((1 << bits) - 1)


def encode_u(value: int, bits: int) -> int:
    if value < 0 or value >= (1 << bits):
        raise ValueError(f"Immediate {value} out of range for {bits}-bit unsigned")
    return value


def normalize_mnemonic(mn: str, spec: dict) -> str:
    mn_u = mn.upper()
    if mn_u in ("MOV", "NOP"):
        return mn_u
    aliases = spec.get("aliases", {})
    return aliases.get(mn_u, mn_u)


def lower_seabird_boot(mnemonic: str, ops: List[str], spec: dict) -> Tuple[str, List[str]]:
    m = mnemonic.upper()
    if m in ("ADDI", "ANDI", "ORI", "XORI") and len(ops) == 2:
        return m, [ops[0], ops[0], ops[1]]
    if m in ("AND", "OR", "XOR", "ADD", "SUB") and len(ops) == 2:
        imm = ops[1].strip()
        if imm.startswith("#") or imm[:1].isdigit() or imm.startswith("-"):
            if m == "AND":
                return "ANDI", [ops[0], ops[0], imm]
            if m == "OR":
                return "ORI", [ops[0], ops[0], imm]
            if m == "XOR":
                return "XORI", [ops[0], ops[0], imm]
            if m == "ADD":
                return "ADDI", [ops[0], ops[0], imm]
            if m == "SUB":
                if imm.startswith("#"):
                    imm = imm[1:]
                return "ADDI", [ops[0], ops[0], f"-{imm}"]
        if m == "AND":
            return "AND", [ops[0], ops[0], ops[1]]
        if m == "OR":
            return "OR", [ops[0], ops[0], ops[1]]
        if m == "XOR":
            return "XOR", [ops[0], ops[0], ops[1]]
        if m == "ADD":
            return "ADD", [ops[0], ops[0], ops[1]]
        if m == "SUB":
            return "SUB", [ops[0], ops[0], ops[1]]
    if m == "ST":
        return "SD", ops
    if m == "STW":
        return "SW", ops
    if m == "STQ":
        return "SD", ops
    if m == "LEA":
        if len(ops) != 2:
            raise ValueError("LEA expects: LEA rd, [base+off]")
        mem = ops[1].strip()
        if mem.startswith("[") and mem.endswith("]"):
            inner = mem[1:-1].replace(" ", "")
            if "+" in inner:
                base, off = inner.split("+", 1)
                return "ADDI", [ops[0], base, off]
            if "-" in inner and not inner.startswith("-"):
                base, off = inner.split("-", 1)
                return "ADDI", [ops[0], base, f"-{off}"]
            if REG_RE.match(inner) or inner.upper() in spec.get("register_aliases", {}):
                return "ADDI", [ops[0], inner, "0"]
            return "ADDI", [ops[0], "R0", inner]
        raise ValueError("LEA requires memory operand")
    if m == "JMP":
        if len(ops) != 1:
            raise ValueError("JMP expects: JMP target")
        if REG_RE.match(ops[0].strip()) or ops[0].strip().upper() in spec.get("register_aliases", {}):
            return "JALR", ["R0", ops[0], "0"]
        return "JAL", ["R0", ops[0]]
    if m == "CALL":
        if len(ops) != 1:
            raise ValueError("CALL expects: CALL target")
        if REG_RE.match(ops[0].strip()) or ops[0].strip().upper() in spec.get("register_aliases", {}):
            return "JALR", ["R1", ops[0], "0"]
        return "JAL", ["R1", ops[0]]
    if m == "RET":
        return "JALR", ["R0", "R1", "0"]
    if m == "JZ":
        return "BEQ", ["R31", "R0", ops[0]]
    if m == "JNZ":
        return "BNE", ["R31", "R0", ops[0]]
    if m == "RDCR":
        return "ADDI", [ops[0], "R0", "0"]
    if m == "WRCR":
        return "ADDI", ["R0", "R0", "0"]
    if m in ("CLI", "STI", "HLT", "PUSHF", "POPF", "MEMFILL"):
        return "ADDI", ["R0", "R0", "0"]
    if m == "RDTS":
        return "ADDI", [ops[0], "R0", "0"]
    if m == "BRR":
        if len(ops) != 1:
            raise ValueError("BRR expects: BRR rs")
        return "JALR", ["R0", ops[0], "0"]
    if m == "PUSH":
        if len(ops) != 1:
            raise ValueError("PUSH expects: PUSH rs")
        return "SD", ["[SP]", ops[0]]
    if m == "POP":
        if len(ops) != 1:
            raise ValueError("POP expects: POP rd")
        return "LD", [ops[0], "[SP]"]
    if m == "TSTI":
        if len(ops) != 2:
            raise ValueError("TSTI expects: TSTI rs, imm")
        return "ANDI", ["R31", ops[0], ops[1]]
    if m == "NOT":
        if len(ops) != 1:
            raise ValueError("NOT expects: NOT rd")
        return "XORI", [ops[0], ops[0], "-1"]
    if m == "SUBI":
        if len(ops) == 2:
            imm = ops[1].strip()
            if imm.startswith("#"):
                imm = imm[1:]
            return "ADDI", [ops[0], ops[0], f"-{imm}"]
        if len(ops) == 3:
            imm = ops[2].strip()
            if imm.startswith("#"):
                imm = imm[1:]
            return "ADDI", [ops[0], ops[1], f"-{imm}"]
        raise ValueError("SUBI expects: SUBI rd, #imm or SUBI rd, rs, #imm")
    if m == "SHR":
        return "SRLI", ops
    return mnemonic, ops


def first_pass(lines: List[str]) -> Tuple[List[AsmLine], Dict[str, int], Dict[str, int]]:
    pcs: List[AsmLine] = []
    labels: Dict[str, int] = {}
    symbols: Dict[str, int] = {}
    pc = 0

    for idx, raw in enumerate(lines, start=1):
        text = COMMENT_RE.sub("", raw).strip()
        if not text:
            continue

        m = LABEL_RE.match(text)
        if m:
            labels[m.group(1)] = pc
            continue

        if text.startswith("."):
            parts = text.split(None, 1)
            directive = parts[0].lower()
            arg = parts[1].strip() if len(parts) > 1 else ""
            if directive == ".org":
                pc = parse_int(arg, symbols)
            elif directive == ".align":
                align = parse_int(arg, symbols)
                if align <= 0:
                    raise ValueError(f".align must be >0 (line {idx})")
                pc = (pc + align - 1) & ~(align - 1)
            elif directive == ".word":
                pc += 4
            elif directive == ".dword" or directive == ".qword":
                pc += 8
            elif directive == ".space":
                pc += parse_int(arg, symbols)
            elif directive in (".equ", ".set"):
                chunks = [c.strip() for c in arg.split(",", 1)]
                if len(chunks) != 2:
                    raise ValueError(f"{directive} expects '.equ NAME, value' (line {idx})")
                name, val = chunks
                symbols[name] = parse_int(val, symbols)
            else:
                # section/mode/macros are accepted but do not affect size here
                pass
            continue

        pcs.append(AsmLine(line_no=idx, text=text, pc=pc))
        pc += 4

    return pcs, labels, symbols


def resolve_imm(token: str, labels: Dict[str, int], symbols: Dict[str, int], pc: int, relative: bool) -> int:
    tok = token.strip()
    if tok in labels:
        target = labels[tok]
        return target - pc if relative else target
    return parse_int(tok, symbols)


def encode_instruction(
    mnemonic: str,
    ops: List[str],
    pc: int,
    labels: Dict[str, int],
    symbols: Dict[str, int],
    spec: dict,
) -> int:
    all_symbols = dict(symbols)
    all_symbols.update(labels)

    if spec.get("profile_mode") == "seabird_boot":
        mnemonic, ops = lower_seabird_boot(mnemonic, ops, spec)

    if mnemonic == "NOP":
        return 0x00000013  # addi x0, x0, 0
    if mnemonic == "MOV":
        if len(ops) != 2:
            raise ValueError("MOV expects: MOV rd, rs")
        rd = parse_reg(ops[0], spec)
        rs1 = parse_reg(ops[1], spec)
        imm = 0
        return ((imm & 0xFFF) << 20) | (rs1 << 15) | (0b000 << 12) | (rd << 7) | 0b0010011

    if mnemonic == "MOVI":
        if len(ops) != 2:
            raise ValueError("MOVI expects: MOVI rd, imm")
        rd = parse_reg(ops[0], spec)
        imm_raw = resolve_imm(ops[1], labels, symbols, pc, relative=False)
        if -2048 <= imm_raw <= 2047:
            imm = encode_signed(imm_raw, 12)
            return (imm << 20) | (0 << 15) | (0b000 << 12) | (rd << 7) | 0b0010011
        imm_31_12 = encode_u((imm_raw >> 12) & 0xFFFFF, 20)
        return (imm_31_12 << 12) | (rd << 7) | 0b0110111

    inst = spec["instructions"].get(mnemonic)
    if not inst:
        raise ValueError(f"Unknown mnemonic '{mnemonic}'")

    fmt = inst["format"]
    opcode = int(inst["opcode"], 2)
    funct3 = int(inst.get("funct3", "0"), 2)
    funct7 = int(inst.get("funct7", "0"), 2)

    if fmt == "R":
        if len(ops) != 3:
            raise ValueError(f"{mnemonic} expects 3 operands")
        rd = parse_reg(ops[0], spec)
        rs1 = parse_reg(ops[1], spec)
        rs2 = parse_reg(ops[2], spec)
        return (funct7 << 25) | (rs2 << 20) | (rs1 << 15) | (funct3 << 12) | (rd << 7) | opcode

    if fmt == "I":
        # memory loads: LD rd, [rs1+imm]
        if inst.get("memory", False):
            if len(ops) != 2:
                raise ValueError(f"{mnemonic} expects 2 operands: rd, [base+off]")
            rd = parse_reg(ops[0], spec)
            rs1, imm_raw = parse_mem_operand(ops[1], all_symbols, spec)
            if spec.get("profile_mode") == "seabird_boot":
                imm = imm_raw & 0xFFF
            else:
                imm = encode_signed(imm_raw, 12)
            return (imm << 20) | (rs1 << 15) | (funct3 << 12) | (rd << 7) | opcode

        # jalr: JALR rd, rs1, imm
        # generic I: op rd, rs1, imm
        if len(ops) != 3:
            raise ValueError(f"{mnemonic} expects 3 operands")
        rd = parse_reg(ops[0], spec)
        rs1 = parse_reg(ops[1], spec)
        imm_raw = resolve_imm(ops[2], labels, symbols, pc, relative=False)

        if "imm_high" in inst:
            if spec.get("profile_mode") == "seabird_boot":
                shamt = imm_raw & 0x1F
            else:
                shamt = encode_u(imm_raw, 5)
            imm_high = int(inst["imm_high"], 2)
            imm = (imm_high << 5) | shamt
        else:
            if spec.get("profile_mode") == "seabird_boot":
                imm = imm_raw & 0xFFF
            else:
                imm = encode_signed(imm_raw, 12)

        return (imm << 20) | (rs1 << 15) | (funct3 << 12) | (rd << 7) | opcode

    if fmt == "S":
        # stores: SD [rs1+imm], rs2
        if len(ops) != 2:
            raise ValueError(f"{mnemonic} expects 2 operands: [base+off], rs2")
        rs1, imm_raw = parse_mem_operand(ops[0], all_symbols, spec)
        rs2 = parse_reg(ops[1], spec)
        if spec.get("profile_mode") == "seabird_boot":
            imm = imm_raw & 0xFFF
        else:
            imm = encode_signed(imm_raw, 12)
        imm_11_5 = (imm >> 5) & 0x7F
        imm_4_0 = imm & 0x1F
        return (imm_11_5 << 25) | (rs2 << 20) | (rs1 << 15) | (funct3 << 12) | (imm_4_0 << 7) | opcode

    if fmt == "B":
        # BEQ rs1, rs2, label
        if len(ops) != 3:
            raise ValueError(f"{mnemonic} expects 3 operands: rs1, rs2, target")
        rs1 = parse_reg(ops[0], spec)
        rs2 = parse_reg(ops[1], spec)
        imm_raw = resolve_imm(ops[2], labels, symbols, pc, relative=True)
        if imm_raw % 2 != 0:
            raise ValueError(f"Branch target must be 2-byte aligned (got {imm_raw})")
        imm = encode_signed(imm_raw, 13)
        bit12 = (imm >> 12) & 0x1
        bit11 = (imm >> 11) & 0x1
        bits10_5 = (imm >> 5) & 0x3F
        bits4_1 = (imm >> 1) & 0xF
        return (
            (bit12 << 31)
            | (bits10_5 << 25)
            | (rs2 << 20)
            | (rs1 << 15)
            | (funct3 << 12)
            | (bits4_1 << 8)
            | (bit11 << 7)
            | opcode
        )

    if fmt == "U":
        if len(ops) != 2:
            raise ValueError(f"{mnemonic} expects 2 operands: rd, imm20")
        rd = parse_reg(ops[0], spec)
        imm_raw = resolve_imm(ops[1], labels, symbols, pc, relative=False)
        imm_31_12 = encode_u((imm_raw >> 12) & 0xFFFFF, 20)
        return (imm_31_12 << 12) | (rd << 7) | opcode

    if fmt == "J":
        # JAL rd, label
        if len(ops) != 2:
            raise ValueError(f"{mnemonic} expects 2 operands: rd, target")
        rd = parse_reg(ops[0], spec)
        imm_raw = resolve_imm(ops[1], labels, symbols, pc, relative=True)
        if imm_raw % 2 != 0:
            raise ValueError(f"Jump target must be 2-byte aligned (got {imm_raw})")
        imm = encode_signed(imm_raw, 21)
        bit20 = (imm >> 20) & 0x1
        bits10_1 = (imm >> 1) & 0x3FF
        bit11 = (imm >> 11) & 0x1
        bits19_12 = (imm >> 12) & 0xFF
        return (
            (bit20 << 31)
            | (bits10_1 << 21)
            | (bit11 << 20)
            | (bits19_12 << 12)
            | (rd << 7)
            | opcode
        )

    raise ValueError(f"Unsupported format '{fmt}' for {mnemonic}")


def second_pass(asmlines: List[AsmLine], labels: Dict[str, int], symbols: Dict[str, int], spec: dict) -> Dict[int, int]:
    words: Dict[int, int] = {}

    for item in asmlines:
        try:
            parts = item.text.split(None, 1)
            mnemonic = normalize_mnemonic(parts[0], spec)
            operand_text = parts[1].strip() if len(parts) > 1 else ""
            ops = split_operands(operand_text)
            word = encode_instruction(mnemonic, ops, item.pc, labels, symbols, spec)
            words[item.pc] = word & 0xFFFFFFFF
        except Exception as exc:
            raise ValueError(f"Line {item.line_no}: {exc} :: '{item.text}'") from exc

    return words


def write_hex(words: Dict[int, int], out_path: pathlib.Path) -> None:
    if not words:
        out_path.write_text("", encoding="utf-8")
        return
    lo = min(words)
    hi = max(words)
    lines: List[str] = []
    pc = lo
    while pc <= hi:
        lines.append(f"{words.get(pc, 0):08x}")
        pc += 4
    out_path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def write_bin(words: Dict[int, int], out_path: pathlib.Path) -> None:
    if not words:
        out_path.write_bytes(b"")
        return
    lo = min(words)
    hi = max(words)
    data = bytearray()
    pc = lo
    while pc <= hi:
        data.extend(struct.pack("<I", words.get(pc, 0)))
        pc += 4
    out_path.write_bytes(bytes(data))


def write_sv(words: Dict[int, int], out_path: pathlib.Path) -> None:
    sorted_words = [words[pc] for pc in sorted(words.keys())]
    lines = [
        "// Auto-generated by tools/assemble_clione64.py",
        "",
        f"localparam int PROGRAM_INST_COUNT = {len(sorted_words)};",
    ]

    if sorted_words:
        elems = ", ".join(f"32'h{w:08x}" for w in sorted_words)
        lines.extend(
            [
                "localparam logic [PROGRAM_INST_COUNT*32-1:0] PROGRAM_INSTR_FLAT = {"
                + elems
                + "};",
                "",
            ]
        )
    else:
        lines.extend(
            [
                "localparam logic [31:0] PROGRAM_INSTR_FLAT = 32'h00000013; // NOP",
                "",
            ]
        )

    for idx, w in enumerate(sorted_words):
        lines.append(f"localparam logic [31:0] INST_{idx:04d} = 32'h{w:08x};")
    lines.append("")
    out_path.write_text("\n".join(lines), encoding="utf-8")


def main() -> int:
    ap = argparse.ArgumentParser(description="Assemble Clione64 test programs from a JSON ISA spec")
    ap.add_argument("input", help="Input assembly file")
    ap.add_argument("-o", "--output", required=True, help="Output file path")
    ap.add_argument(
        "--spec",
        default="tools/isa/clione64_isa.json",
        help="ISA JSON specification path",
    )
    ap.add_argument(
        "--format",
        choices=["hex", "bin", "sv"],
        default="hex",
        help="Output format",
    )
    args = ap.parse_args()

    spec_path = pathlib.Path(args.spec)
    in_path = pathlib.Path(args.input)
    out_path = pathlib.Path(args.output)

    spec = load_spec(spec_path)
    lines = in_path.read_text(encoding="utf-8").splitlines()

    asmlines, labels, symbols = first_pass(lines)
    words = second_pass(asmlines, labels, symbols, spec)

    out_path.parent.mkdir(parents=True, exist_ok=True)
    if args.format == "hex":
        write_hex(words, out_path)
    elif args.format == "bin":
        write_bin(words, out_path)
    else:
        write_sv(words, out_path)

    print(f"Assembled {len(words)} instructions -> {out_path}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
