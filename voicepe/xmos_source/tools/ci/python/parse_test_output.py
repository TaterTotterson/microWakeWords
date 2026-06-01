#!/usr/bin/env python
# Copyright 2021-2023 XMOS LIMITED.
# This Software is subject to the terms of the XMOS Public Licence: Version 1.

import argparse

from file_regex_parser import file_regex_parser

all_regex = r"^Tile\[(\d{1})\]\|FCore\[(\d{1})\]\|(\d+)\|(\w+)\|(.+)$"
test_regex = r"^Tile\[(\d{1})\]\|FCore\[(\d{1})\]\|(\d+)\|TEST\|(.+)$"
tile0_regex = r"^Tile\[0\]\|FCore\[(\d{1})\]\|(\d+)\|(\w+)\|(.+)$"
tile1_regex = r"^Tile\[1\]\|FCore\[(\d{1})\]\|(\d+)\|(\w+)\|(.+)$"

def parse_arguments():
    parser = argparse.ArgumentParser(description=("Tool to validate and separate RTOS driver test output"))
    parser.add_argument("infile", help="Input rpt file")
    parser.add_argument("-outfile", default="out", help="Output file name")
    parser.add_argument("--split", default=False, action="store_true", help="Outputs report per tile")
    parser.add_argument("--verbose", default=False, action="store_true", help="Enable verbose printing")
    parser.add_argument("--print_test_results", default=False, action="store_true", help="Print test result list contents")

    parser.parse_args()
    args = parser.parse_args()

    return args

def write_to_file(filename, list):
    f = open(filename, "w")

    for each in list:
        f.write(each)
    f.close()

def main(args):
    frp = file_regex_parser(test_regex, args.infile)
    frp.parse_file(verbose=args.verbose)
    test_list = frp.get_list()

    frp.change_regex(all_regex)
    frp.parse_file(verbose=args.verbose)
    full_list = frp.get_list()

    outfile = args.outfile + ".txt"
    outfile_full = args.outfile + "_full.txt"

    write_to_file(outfile, test_list)
    write_to_file(outfile_full, full_list)

    if args.split:
        frp.change_regex(tile0_regex)
        frp.parse_file(verbose=args.verbose)
        t0_list = frp.get_list()

        frp.change_regex(tile1_regex)
        frp.parse_file(verbose=args.verbose)
        t1_list = frp.get_list()

        outfile_t0 = args.outfile + "_tile0.txt"
        outfile_t1 = args.outfile + "_tile1.txt"

        write_to_file(outfile_t0, t0_list)
        write_to_file(outfile_t1, t1_list)

    if args.print_test_results:
        tmpstr = ""
        for each in test_list:
            tmpstr += each
        print(tmpstr)

if __name__ == "__main__":
    args = parse_arguments()

    main(args)
