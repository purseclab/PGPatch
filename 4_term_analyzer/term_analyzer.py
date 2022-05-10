# Author: Hyungsub Kim
# Email: kim2956@purdue.edu
# Date: October 2, 2020
#
# Goal: Classify each term into one of the followings:
### (1) configuration parameter
### (2) function
### (3) variable
### (4) the RV's physical state
#
# How to execute?
### python term_analyzer.py -s ArduPilot
### python term_analyzer.py -s PX4
### python term_analyzer.py -s Paparazzi
#
# Output format: [Term, Matched type]

import sys, getopt
import os
import pyparsing as pp
from pyparsing import *
import re
import xml.etree.ElementTree as ET
from xml.etree.ElementTree import Element, dump, ElementTree

term_list = []
matched_list = []

def main(argv):
    # ---------------- (Start) Parse command line arguments ----------------
    rv_sw_type = ""
    try:
        opts, args = getopt.getopt(argv, "h:s:")

    except getopt.GetoptError:
        print("[Usage] term_analyzer.py -s <ArduPilot/PX4/Paparazzi> when 's' denote 'RV control software'")
        sys.exit(2)

    for opt, arg in opts:
        if opt == '-h':
            print("[Usage] term_analyzer.py -s <ArduPilot/PX4/Paparazzi> when 's' denote 'RV control software'")
            sys.exit()
        elif opt in ("-s"):
            rv_sw_type = arg

    print("[RV Software Type] is", rv_sw_type)
    # ---------------- (End) Parse command line arguments ----------------

    # 1) Read '3a_expression_tree.txt' file
    Terms = ""
    Term_list = ""
    dirname = os.path.dirname(__file__)
    filename = "../" + "/3a_expression_tree.txt"
    print(dirname)
    filename = os.path.join(dirname, filename)

    with open(filename) as file_read:
        line = file_read.readline()
        while line:
            if "###" in line:
                line = file_read.readline()
            else:
                Terms += line
                Term_list += line
                line = file_read.readline()

    print("---------- (Start)[Terms] ----------")
    print("--- Results of read the txt file ---")
    print(Terms)
    Terms = Terms.replace('\n','')
    result = [x.strip() for x in Terms.split(',')]

    print("--- Results of split (length: ", len(result), ") ---")
    print(result)
    print("---------- (End) [Terms] ----------")
    file_read.close()

    # 2) When the selected RV software is ArduPilot
    special_characters = "^ = ~ < > >= <="
    value_characters = "true false error on"
    print("---------- (Start) [Matching] ----------")
    for term in result:
        matched = False

        # Let's skip if the term is just numbers/temporal logic operators.
        if term.isdigit():
            continue
        if term in special_characters:
            continue
        if term in value_characters:
            continue
        if re.search(r"\^+[0-9]",term):
            continue
        if re.search(r"\@+[0-9]",term):
            continue

        print(term)

        ### 2-1) Try to match a term with a list of configuration parameters
        iteration_config = 0
        if rv_sw_type == "ArduPilot":
            iteration_config = 5
        else:
            iteration_config = 1

        for iter in range(iteration_config):
            dirname = os.path.dirname(__file__)

            if rv_sw_type == "ArduPilot":
                if iter == 0:
                    filename = "./" + "ArduPilot" + "/1_configuration_parameter_list_antenna_tracker.txt"
                elif iter == 1:
                    filename = "./" + "ArduPilot" + "/1_configuration_parameter_list_copter.txt"
                elif iter == 2:
                    filename = "./" + "ArduPilot" + "/1_configuration_parameter_list_plane.txt"
                elif iter == 3:
                    filename = "./" + "ArduPilot" + "/1_configuration_parameter_list_rover.txt"
                elif iter == 4:
                    filename = "./" + "ArduPilot" + "/1_configuration_parameter_list_submarine.txt"

                #print("[DEBUG] ", dirname)
                filename = os.path.join(dirname, filename)
            elif rv_sw_type == "PX4":
                filename = "./" + "PX4" + "/1_configuration_parameter_list_copter.txt"
            elif rv_sw_type == "Paparazzi":
                filename = "./" + "Paparazzi" + "/1_configuration_parameter_list_copter.txt"

            with open(filename) as file_read:
                line = file_read.readline()
                while line:
                    #print("[DEBUG] ", term, "vs.", line)
                    if (term.strip()) == (line.strip()):
                        print("[DEBUG] the term (", term, ") is a configuration parameter.")
                        term_list.append(term.replace(" ", ""))
                        matched_list.append("configuration_parameter")
                        matched = True

                    line = file_read.readline()

            file_read.close()

        ### 2-2) Try to match a term with a list of the RV's physical states
        if matched == False:
            dirname = os.path.dirname(__file__)
            filename = "./" + rv_sw_type + "/2_physical_state_list.txt"
            filename = os.path.join(dirname, filename)

            with open(filename) as file_read:
                line = file_read.readline()
                while line:
                    #print("[DEBUG] ", term, "vs.", line)
                    if (term.strip()) in (line.strip()):
                        print("[DEBUG] the term (", term, ") is the RV's physical states.")
                        term_list.append(term.replace(" ", ""))
                        matched_list.append("physical_state")
                        matched = True

                    line = file_read.readline()

            file_read.close()

        ### 2-3) Try to match a term with a list of functions
        if matched == False:
            dirname = os.path.dirname(__file__)
            filename = "./" + rv_sw_type + "/3_function_list.txt"
            filename = os.path.join(dirname, filename)

            with open(filename) as file_read:
                line = file_read.readline()
                while line:
                    #print("[DEBUG] ", term, "vs.", line)
                    if (term.strip()) in (line.strip()):
                        print("[DEBUG] the term (", term, ") is a function.")
                        term_list.append(term.replace(" ", ""))
                        matched_list.append("function")
                        matched = True
                        break

                    line = file_read.readline()

            file_read.close()

        ### 2-4) Try to match a term with a list of variable
        if matched == False:
            dirname = os.path.dirname(__file__)
            filename = "./" + rv_sw_type + "/4_variable_list.txt"
            filename = os.path.join(dirname, filename)

            with open(filename) as file_read:
                line = file_read.readline()
                while line:
                    #print("[DEBUG] ", term, "vs.", line)
                    if (term.strip()) in (line.strip()):
                        print("[DEBUG] the term (", term, ") is a variable.")
                        term_list.append(term.replace(" ", ""))
                        matched_list.append("variable")
                        matched = True
                        break

                    line = file_read.readline()

            file_read.close()

    print("---------- (End) [Matching] ----------")

    print("---------- (Start) [Result] ----------")
    dirname = os.path.dirname(__file__)
    filename = "../" + "3b_term_classification_table.txt"
    filename = os.path.join(dirname, filename)
    f = open(filename, "w")
    f.write("### Goal: Classify each term into one of the followings:\n")
    f.write("### (1) configuration parameter\n")
    f.write("### (2) function\n")
    f.write("### (3) variable\n")
    f.write("### (4) the RV's physical state\n")
    f.write("### Output format: [Term, Matched type] ###\n")
    f.write("### '###'denotes a comment. ###\n")

    each_term = Term_list.split("\n")
    print(each_term)

    for i in range(len(term_list)):
        print(term_list[i] + " is " + matched_list[i])
        #f.write(term_list[i] + "," + matched_list[i] + "\n")
        f.write(each_term[i] + matched_list[i] + "\n")
    print("---------- (End) [Result] ----------")
    f.close()
if __name__ == "__main__":
   main(sys.argv[1:])