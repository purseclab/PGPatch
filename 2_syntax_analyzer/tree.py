# Author: Hyungsub Kim
# Email: kim2956@purdue.edu
# Date: October 2, 2020
# Goal: Converting results of pyparsing into a tree
# Output format: [Tree level, operator, left node, center node, right node]

terminal_node = 0
tree_print = ""

class BinaryTree(object):
    def __init__(self, left=None, data=None, right=None):
         self.left = left
         self.data = data
         self.right = right

def create_tree(data, initiative, tree_level, node, parent_node, filename, verbose):

    global terminal_node
    global root
    global tree_print

    # Open a txt file to store the tree
    f = open(filename, "a")

    if initiative == True:
        tree_level = 0
        terminal_node = 0
        tree_print = ""

    if not data:
        return data

    #print("Parent_node:", parent_node)

    #print('# of lists in list:', len(data))
    if type(data) != int and len(data) != 3:
        #if len(data) > 3:
            #print(" ^ terminal node")
        return data

    if type(data) != int and len(data) == 3:
        l, d, r = data

        #print("[DEBUG] len(Left):", len(l), ", len(Center):", len(d), ", len(Right):", len(r))
        #print("[DEBUG] type(Left):", type(l), ", type(Center):", type(d), ", type(Right):", type(r))

        p_node = parent_node

        if type(l) != int and type(d) != int and type(r) != int:
            print_line = "(" + str(node) + " tree level):" + str(tree_level) + ", operator:" + str(d) + ", Left:" + str(l) + ", Right:" + str(r) + "\n"
            print(print_line)

            if verbose == "yes":
                print_line = "###" + print_line
                f.write(print_line)

            #tree_level += 1
            #print("Left:", len(l), ", Center:", len(d), ", Right:", len(r))
            parent_node = d

        if len(l) > 3:
            print_line = "[Parent node of the terminal node]" + "tree level:" + str(tree_level) + "Center:" + str(d) + "(" + str(node) + ")" + "\n"
            print(print_line)

            if verbose == "yes":
                print_line = "###" + print_line
                f.write(print_line)

            print_line = "\t\t[Terminal node]" + "tree level:" + str(tree_level+1) + "Left:" + str(l) + ", Right:" + str(r) + "(" + str(node) + ")" + "\n"
            print(print_line)

            if verbose == "yes":
                print_line = "###" + str(print_line)
                f.write(print_line)

            terminal_node += 1

            line = str(tree_level) + ", " + p_node + ", " + str(l) + ", " + str(d) + ", " + str(r) + ", " + "\n"
            f.write(line)
            parent_node = d

        return BinaryTree(create_tree(l, False, tree_level+1, "left", parent_node, filename, verbose), create_tree(d, False, tree_level-1, "center", parent_node, filename, verbose), create_tree(r, False, tree_level+1, "right", parent_node, filename, verbose))



