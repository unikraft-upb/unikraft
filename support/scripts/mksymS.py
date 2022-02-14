#!/usr/bin/env python3
#
# Copyright (c) 2021, Karlsruhe Institute of Technology (KIT)
#
# This program is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 2 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
# General Public License for more details.

import argparse
import subprocess
import sys
import shutil
import re

class Symbol:
    def __init__(self, addr, name):
        self.addr = addr
        self.name = name

def main():
    parser = argparse.ArgumentParser(
        description='Creates an assembler symbol table from an nm symbol file')
    parser.add_argument('source', help='Symbol file created with nm')
    parser.add_argument('output', help='Assembler file')
    parser.add_argument('-n', '--no-names', action='store_true',
                        help='Omit symbol names and just save addresses')
    opt = parser.parse_args()

    with open(opt.source, 'rt') as fin, open(opt.output, 'wt') as fout:
        symbols = []

        # Match lines of the form:
        # 000000000010503f t _libkvmplat_start64
        # We only import function symbols for now (t=local, T=global function)
        symfilter = re.compile(r'^([0-9,a-f]+)\s[t|T]\s(.+)$')
        for line in fin.readlines():
            sym = symfilter.match(line)
            if (sym is not None):
                symbols.append(Symbol(int(sym.group(1), 16), sym.group(2)))

        # Sort the symbols by address. This way we can perform binary search in
        # unikraft when resolving a symbol
        symbols.sort(key=lambda x: x.addr)

        # Build the address table. We subtract each addr from the address of
        # the entry itself. This way, address information in the table is
        # position independent and we need only 4 bytes. This works as long as
        # an entry is not more than 2GiB away from its symbol.
        fout.write('.section ".uk_symtab", "a"\n')
        fout.write('.global _uk_symtab_addrs_start\n')
        fout.write('_uk_symtab_addrs_start:\n')
        for sym in symbols:
            fout.write('\t.long . - {} /* {} */\n'.format(hex(sym.addr),
                sym.name))
        fout.write('.global _uk_symtab_addrs_end\n')
        fout.write('_uk_symtab_addrs_end:\n')
        fout.write('\n')

        # Build the name table. Each entry stores the length of the string in
        # the first byte. The next bytes encode the null-terminated string.
        # This makes seeking to a certain string faster than storing
        # null-terminating strings only.
        fout.write('.global _uk_symtab_names_start\n')
        fout.write('_uk_symtab_names_start:\n')

        if not opt.no_names:
            for sym in symbols:
                l = len(sym.name)
                if (l > 255):
                    print('Symbol too long:', sym.name)
                    sys.exit(1)
                line  = '\t.byte\t'
                line += '0x{:02x}'.format(l)
                line += ''.join(', 0x{:02x}'.format(ord(c)) for c in sym.name)
                line += ', 0x00 /* {} */\n'.format(sym.name)
                fout.write(line)

        fout.write('.global _uk_symtab_names_end\n')
        fout.write('_uk_symtab_names_end:\n')

if __name__ == '__main__':
    main()
