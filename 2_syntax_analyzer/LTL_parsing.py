# Author: Hyungsub Kim
# Email: kim2956@purdue.edu
# Date: October 2, 2020
#
# Goal: (1) Parsing LTL formulas (2) Converting a given formula into a '*.txt' file
#
# How to execute?
### python LTL_parsing.py -v yes
### python LTL_parsing.py -v no
#
# Output format: [Tree level, operator, left node, center node, right node]

import sys, getopt
import os
from tree import create_tree
import pyparsing as pp
from pyparsing import *
import re
import xml.etree.ElementTree as ET
from xml.etree.ElementTree import Element, dump, ElementTree

def main(argv):
    # ---------------- (Start) Parse command line arguments ----------------
    verbose = ""
    try:
        opts, args = getopt.getopt(argv, "h:v:")

    except getopt.GetoptError:
        print("[Usage] LTL_parsing.py -v <yes/no> when 'v' denote 'verbose'")
        sys.exit(2)

    for opt, arg in opts:
        if opt == '-h':
            print("[Usage] LTL_parsing.py -verbose <yes/no>")
            sys.exit()
        elif opt in ("-v"):
            verbose = arg

    print("[Verbose] is", verbose)
    # ---------------- (End) Parse command line arguments ----------------

    ppc = pyparsing_common

    ParserElement.enablePackrat()
    sys.setrecursionlimit(3000)

    integer = ppc.integer

    # One or more contiguous characters; construct with a string containing
    # the set of allowed initial characters, and an optional second string of allowed body characters
    variable = Word( "_" + alphas + "_" + "+" + "-" + "*" + "\\" + ".", alphanums + "_" + "+" + "-" + "*" + "\\" + "." )
    operand = integer | variable

    # To use the infixNotation helper:
    #   1.  Define the "atom" operand term of the grammar.
    #       For this simple grammar, the smallest operand is either
    #       and integer or a variable.  This will be the first argument
    #       to the infixNotation method.
    #   2.  Define a list of tuples for each level of operator
    #       precendence.  Each tuple is of the form
    #       (opExpr, numTerms, rightLeftAssoc, parseAction), where
    #       - opExpr is the pyparsing expression for the operator;
    #          may also be a string, which will be converted to a Literal
    #       - numTerms is the number of terms for this operator (must
    #          be 1 or 2)
    #       - rightLeftAssoc is the indicator whether the operator is
    #          right or left associative, using the pyparsing-defined
    #          constants opAssoc.RIGHT and opAssoc.LEFT.
    #       - parseAction is the parse action to be associated with
    #          expressions matching this operator expression (the
    #          parse action tuple member may be omitted)
    #   3.  Call infixNotation passing the operand expression and
    #       the operator precedence list, and save the returned value
    #       as the generated pyparsing expression.  You can then use
    #       this expression to parse input strings, or incorporate it
    #       into a larger, more complex grammar.
    #

    expr = pp.oneOf("G N") + infixNotation(
        operand,
        [
            ("^1", 2, opAssoc.LEFT),
            ("^2", 2, opAssoc.LEFT),
            ("^3", 2, opAssoc.LEFT),
            ("^4", 2, opAssoc.LEFT),
            ("^5", 2, opAssoc.LEFT),
            ("@1", 2, opAssoc.LEFT),
            ("@2", 2, opAssoc.LEFT),
            ("@3", 2, opAssoc.LEFT),
            ("@4", 2, opAssoc.LEFT),
            ("@5", 2, opAssoc.LEFT),
            ("~", 2, opAssoc.RIGHT),
            ("=", 2, opAssoc.LEFT),
            (">", 2, opAssoc.RIGHT),
            ("<", 2, opAssoc.RIGHT),
            (">=", 2, opAssoc.RIGHT),
            ("<=", 2, opAssoc.RIGHT),
            ("!=", 2, opAssoc.LEFT),
        ],
    )

    print("--------------------------------- Symbol Description ---------------------------------")
    print("[Temporal operator] G: always, N: never, ~: imply")
    print("[Conjunction] ^n: and, @n: or where 'n' denotes number (e.g., ^1: the first ^ and ^2: the second ^)")
    print("[Verb] >: greater than symbol, <: less than symbol")
    print("[Verb] >=: greater than or equal to symbol, <=: less than or equal to symbol")
    print("[Verb] =: equal, !=: not equal");
    print("--------------------------------------------------------------------------------------")

    test = [
        "G ((GPS_loss = true) ^1 (Loss_time > COM_POS_FS_DELAY)) ~ (GPS_fail = on)",
        "G ((armed = false) ^1 (SAIL_ENABLE = 1) ^2 (WNDVN_TYPE = 0)) ~ (pre_arm_checks = error)",
        "G (Failsafe = on) ^1 ((FS_BATT_ENABLE = 2) ^2 (home_distance >= 2)) ~ (mode = RTL)",
        "G ((mode = LAND) @1 (mode = RTL)) ~ (_constraints.tilt_t = _constraints.tilt_t_1)",
        "G (ALT_current <= RTL_alt) ^1 (MODE_current = RTL) ~ (ALT_previous <= ALT_current)",
        "G (MainALT_src = Barometer) ~ (ALT_current = ALT_barometer) ^1 (ALT_current != ALT_gps)",
        "G (GPS_fail = on) ^1 (Barometer = on) ~ (ALT_src = Barometer)",
        "G ((MODE_t = ALTHOLD) @1 (THROTTLE_t = middle) ^1 (Term1 = Term2)) ~ (ALT_t = ALT_t_1)",
        "G ((vel_variance >= (fs_ekf_thresh*2)) ~ ((over_thresh_count_t = over_thresh_count_t+2)))",
    ]

    print("--------------------------------- (Start-testing) LTL formulas parsing ---------------------------------")
    cnt = 1
    for t in test:
        dirname = os.path.dirname(__file__)
        filename = "./" + "test/LTL_formula_" + str(cnt) + ".txt"
        print(dirname)
        filename = os.path.join(dirname, filename)

        f = open(filename, "w")
        f.write("### (1) A given formula ###" + str(cnt) + ", " + t + "\n")
        f.write("### (2) Output format: [Tree level, operator, left node, center node, right node] ###\n")
        f.close()

        print("Temporal logic formula: ", t)
        result = expr.parseString(t)
        print("Parsing result: ", result)

        print("################################")
        print("Tree: ")
        print(create_tree(result[1], True, 0, "root", "root", filename, verbose))
        print("################################")
        print("")
        cnt = cnt + 1
    print("---------------------------------- (End-testing) LTL formulas parsing ----------------------------------")

    print("--------------------------------- (Start) LTL formulas parsing ---------------------------------")

    # Read a given formula
    dirname = os.path.dirname(__file__)
    filename = "../" + "/1a_formula.txt"
    print(dirname)
    filename = os.path.join(dirname, filename)

    #file_read = open(filename, "r")
    #t = file_read.read()
    #file_read.close()

    with open(filename) as file_read:
        line = file_read.readline()
        while line:
            if "###" in line:
                line = file_read.readline()
            else:
                t = line
                break
    print("[DEBUG]", t)
    file_read.close()

    # Open a txt file to store the parsing result
    dirname = os.path.dirname(__file__)
    filename = "../" + "/3a_expression_tree.txt"
    print(dirname)
    filename = os.path.join(dirname, filename)

    f = open(filename, "w")
    f.write("### (1) A given formula ###" + "\n" + "### " + t)
    f.write("### (2) Output format: [Tree level, operator, left node, center node, right node] ###\n")
    f.write("### '###'denotes a comment. ###\n")
    f.close()

    print("Temporal logic formula: ", t)
    result = expr.parseString(t)
    print("Parsing result: ", result)

    print("################################")
    print("Tree: ")
    print(create_tree(result[1], True, 0, "root", "root", filename, verbose))
    print("################################")
    print("")

    print("---------------------------------- (End) LTL formulas parsing ----------------------------------")


if __name__ == "__main__":
   main(sys.argv[1:])