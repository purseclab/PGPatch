#!/usr/bin/python3

# Author: Hyungsub Kim
# Email: kim2956@purdue.edu
# This is a XML parser for Paparazzi.
# It parses the following properties from XML files:
#   (1) Parameter name
#   (2) Variable name on source code
#   (3) Minimum parameter value
#   (4) Maximum parameter value

# Usage: xml_parse_paparazzi.py -i <inputfile> -o <outputfile>
# (e.g.,) python xml_parse_paparazzi.py -i ./ctl_adaptive.xml -o ./ctl_adaptive.csv
# (e.g.,) python xml_parse_paparazzi.py -i ./ctl_adaptive_h_ff.xml -o ./ctl_adaptive_h_ff.csv
# (e.g.,) python xml_parse_paparazzi.py -i ./ctl_basic.xml -o ./ctl_basic.xml.csv

# Result:
# [Parameter_name;;Variable;;Min;;Max]
# (e.g.,) H_CTL_ROLL_KFFA;;h_ctl_roll_Kffa;;0;;5000;;
#
# How to get such XML file from Paparazzi?
# https://github.com/paparazzi/paparazzi/tree/a32fc33bf8abecfd767972d32d5720f8763a5fe7/conf/settings

import sys, getopt
import csv
import re
import xml.etree.ElementTree as ET
from xml.etree.ElementTree import Element, dump, ElementTree

def main(argv):
    # Parse command line arguments (i.e., input and output file)
    
    output_file_type = 0 # -1: txt, 1: csv
    inputfile = ''
    outputfile = ''

    try:
        opts, args = getopt.getopt(argv,"hi:o:",["ifile=","ofile="])

    except getopt.GetoptError:
        print("xml_parse_paparazzi.py -i <inputfile> -o <outputfile>")
        sys.exit(2)
    
    for opt, arg in opts:
        if opt == '-h':
            print("xml_parse_paparazzi.py -i <inputfile> -o <outputfile>")
            sys.exit()
        elif opt in ("-i", "--ifile"):
            inputfile = arg
        elif opt in ("-o", "--ofile"):
            outputfile = arg
    print('Input file is "', inputfile)
    print('Output file is "', outputfile)


    if re.search(".txt", outputfile):
        output_file_type = -1
        
        store_file = open(outputfile, 'w')
        store_file.close()

        store_file = open(outputfile, 'a')

    elif re.search(".csv", outputfile):
        output_file_type = 1
        store_file = open(outputfile, 'w')
        store_file.close()

        store_file = open(outputfile, 'a')
        # creating a csv writer object
        csv_writer = csv.writer(store_file) 
        
        # writing the fields 
        fields = ['Parameter name', 'Variable', 'Min', 'Max'] 
        csv_writer.writerow(fields) 

    else:
        print("The output file can be either \'.txt\' or \'.csv\'.")
        sys.exit(2)



    doc = ET.parse(inputfile)

    # Load root node
    root = doc.getroot()

    count = 1      
 
    for parameters in root.iter("dl_settings"):
    
        if parameters.get("NAME") == None:
            continue

        for object in parameters.iter("dl_setting"):
            
            Max = ""
            Min = ""
            Var = ""
            Shortname = ""
            Param = ""

            Max = object.get("MAX")
            Min = object.get("MIN")
            Var = object.get("VAR")
            Param = object.get("param")
            
            if Max == None:
                Max = ""
            if Min == None:
                Min = ""
            if Var == None:
                Var = ""
            if Param == None:
                Param = ""
            
            print("%d, parameter name: %r, Var: %r, Min: %r, Max: %r" % (count, Param, Var, Min, Max))
            
            count = count +1
            
            if output_file_type == -1:
                write = ";;".join([Param, Var, Min, Max, "\n"]).encode('utf-8')
                store_file.write(write)

            elif output_file_type == 1:
                # writing the data rows 
                write = ";;".join([Param, Var, Min, Max]).encode('utf-8')
                row = write.split(';;')
                csv_writer.writerow(row)

            print("[Parameter name, Variable, Min, Max]")
            

if __name__ == "__main__":
   main(sys.argv[1:])
