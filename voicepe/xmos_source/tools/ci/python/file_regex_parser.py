# Copyright 2021-2023 XMOS LIMITED.
# This Software is subject to the terms of the XMOS Public Licence: Version 1.

import re

class file_regex_parser:
    '''
    This class takes a file and creates a list of lines that match the provided regex.

    Note: Multiline regexes are not supported.

        Parameters:
            regex (str): A regex to use to match each line against
            infile (str): A filepath to the file to parse

        Returns:
            A file_regex_parser object
    '''

    def __init__(self, regex, infile):
        self.list_parsed = []
        self.regex = regex
        self.infile = infile

    def parse_file(self, verbose=False):
        self.list_parsed = []
        f = open(self.infile, "r")

        while 1:
            line = f.readline()

            if len(line) == 0:
                if verbose:
                    print("Parse complete")
                break

            p = re.match(self.regex, line)

            if p:
                if verbose:
                    print(p.group(0))
                self.list_parsed.append(line)

        f.close()
        return

    def change_regex(self, regex):
        self.regex = regex

    def change_file(self, infile):
        self.infile = infile

    def get_list(self):
        return self.list_parsed

    def __str__(self):
        retstr = ""
        for each in self.list_parsed:
            retstr += each
        return retstr

    def __len__(self):
        return len(self.list_parsed)
