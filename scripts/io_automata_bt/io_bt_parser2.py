import pyparsing
import sys
print "#############################################################"
print "#############################################################"


################################################################################
# global variables & initialization
################################################################################

exp_n       = "7"
io_filename = "sampleIO" + exp_n + ".txt"
bt_filename = "sampleBT" + exp_n + ".txt"
f = open(io_filename, 'r')

n_states  = int(f.readline())
q_init    = int(f.readline())
sigma_in  = range(0, 2)         # check this --> 0: fail, 1: succ
sigma_out = range(1, int(f.readline()) + 1 )
d_transi  = [[ None for x in range(2)] for y in range(n_states)]
d_action  = [[ None for x in range(2)] for y in range(n_states)]
states    = range(1, n_states + 1)
# 0: success, 1: failure for d_transi and d_action

for i in range(n_states):
    d_transi[i] = map(int, f.readline().split())
for i in range(n_states):
    d_action[i] = map(str, f.readline().split())

# print n_states
# print q_init
# print sigma_in
# print sigma_out
print "d_transi:", d_transi
print "d_action:", d_action

# pyparsing objects for behavior tree string simplification
cont_seq = pyparsing.Word(pyparsing.alphanums) | '+' | '-' | '{' | '}'
nest_seq = pyparsing.nestedExpr( '[', ']', content=cont_seq)
cont_sel = pyparsing.Word(pyparsing.alphanums) | '+' | '-' | '[' | ']'
nest_sel = pyparsing.nestedExpr( '{', '}', content=cont_sel)


################################################################################
# low level functions
################################################################################

# check that the first transition (action) is independent of the
# last action success or failure, and goes to the same state.
def check_initial(q_init):
    idx_qi = q_init - 1
    if d_transi[idx_qi][0] == d_transi[idx_qi][1] and \
       d_action[idx_qi][0] == d_action[idx_qi][1]:
        print "initial check:", True
        return True
    else:
        return False

# check that the d_transi and d_action matrices have only 2 states that are
# not 'None', the remaining should be q_init and q_final where d_cation[q_init]
# holds the parsed bt string from the io automaton
def check_parse_result(q_init):
    idx_qi = q_init - 1

    if d_transi[idx_qi][0] == d_transi[idx_qi][1] and \
       d_transi[idx_qi][0] != None:
        q_final = d_transi[idx_qi][0]
        idx_qf = q_final - 1
        print "q_final", q_final
    else:
        print "catch error parsing 1"
        sys.exit(1)

    if d_transi[idx_qf][0] == d_transi[idx_qf][1] and \
       d_transi[idx_qf][0] == q_init:
        print "checked initial and last states", q_init, q_final
    else:
        print "catch error parsing 2"
        sys.exit(1)

    for i in range(n_states):
        if i != idx_qi and i != idx_qf:
            if d_transi[i] == [None, None] and \
               d_action[i] == [None, None]:
                print "state", i, "is fine"
            else:
                print "state", i, "is not fine"
                print "catch error parsing 3"
                sys.exit(1)

    parsed_bt_string = d_action[idx_qi][0]
    print "io automata parsed successfully"
    print "bt string:", parsed_bt_string
    return parsed_bt_string

# p: prev_success_failure, first time could be 1 or 0 (try both),
# after that, all others should be 0 for sequence / 1 for selector
def check_sequence(q_1, seq_action, seq_state, q_f, p):
    idx_q1 = q_1 - 1            # hacer esto para 0 y 1 a_1 y q_2
    a_1 = d_action[idx_q1][p]   # deberia ser para cualquier 0/1 solo el primero
    q_2 = d_transi[idx_q1][p]   # deberia ser para cualquier 0/1
    idx_q2 = q_2 - 1            # q_f es el que ya no pertenece a sequence
    a_2 = d_action[idx_q2][0]   # 0 because for sequence is success
    q_3 = d_transi[idx_q2][0]   # 0 because for sequence is success
    idx_q3 = q_3 - 1
    a1_3 = d_action[idx_q2][1]  # 1 because for sequence is failure
    a2_3 = d_action[idx_q3][1]
    q1_4 = d_transi[idx_q2][1]
    q2_4 = d_transi[idx_q3][1]
    # print a1_3, a2_3
    # print q1_4, q2_4
    if a1_3 == a2_3 and q1_4 == q2_4: # check the actions and state are the same
        print "action", a_1, "is in sequence with action", a_2
        seq_action.append(a_2)
        seq_state.append (q_2)
        check_sequence(q_2, seq_action, seq_state, q_f, 0) # 0: prev = success
    else:
        # print "action", a_1, "is NOT in sequence with action", a_2
        # print "adding last state", q_2
        q_f.append(q_2)
    return a_1, q_1

# p: prev_success_failure, first time could be 1 or 0 (try both),
# after that, all others should be 0 for sequence / 1 for selector
def check_selector(q_1, sel_action, sel_state, q_f, p):
    idx_q1 = q_1 - 1            # hacer esto para 0 y 1 a_1 y q_2
    a_1 = d_action[idx_q1][p]   # deberia ser para cualquier 0/1 solo el primero
    q_2 = d_transi[idx_q1][p]   # deberia ser para cualquier 0/1
    idx_q2 = q_2 - 1            # q_f es el que ya no pertenece a selector
    a_2 = d_action[idx_q2][1]   # 1 because for selector is failure
    q_3 = d_transi[idx_q2][1]   # 1 because for selector is failure
    idx_q3 = q_3 - 1
    a1_3 = d_action[idx_q2][0]  # 0 because for selector is success
    a2_3 = d_action[idx_q3][0]
    q1_4 = d_transi[idx_q2][0]
    q2_4 = d_transi[idx_q3][0]
    # print a1_3, a2_3
    # print q1_4, q2_4
    if a1_3 == a2_3 and q1_4 == q2_4: # check the actions and state are the same
        print "action", a_1, "is in selector with action", a_2
        sel_action.append(a_2)
        sel_state.append (q_2)
        check_selector(q_2, sel_action, sel_state, q_f, 1) # 1: prev = failure
    else:
        # print "action", a_1, "is NOT in selector with action", a_2
        # print "adding last state", q_2
        q_f.append(q_2)
    return a_1, q_1


################################################################################
# middle level functions
################################################################################

# check for a given state and prev_succ_fail if it begins a sequence, clearly
# it could be a subset of the whole sequence, but hopefully this problem will
# disappear after simplifying a finite number of times
def check_state_seq(q_check, prev_action_out):
    seq_action = []
    seq_state  = []
    q_f        = []
    [first_action, first_state] = check_sequence(q_check, seq_action, seq_state,
                                                 q_f    , prev_action_out)
    if len(seq_action) == 0:
        # print "bye"
        return
    seq_action.insert(0, first_action)
    seq_state.insert (0, first_state )
    comp_action = ' '.join(seq_action)
    comp_action = '[' + comp_action + ']'

    # print "seq_action, seq_state, q_f", seq_action, seq_state, q_f
    # print comp_action

    for idx, q in enumerate(seq_state):
        if idx == 0:            # checkear que pasa cuando no hay ninguno
            print "replacing action", d_action[q-1][prev_action_out], \
                "with", comp_action, "in state", q
            d_transi[q-1][prev_action_out] = q_f[0]
            d_action[q-1][prev_action_out] = comp_action
        else:
            d_transi[q-1][:] = [None, None]
            d_action[q-1][:] = [None, None]

    print "d_transi from check_state_seq", d_transi
    print "d_action from check_state_seq", d_action

# check for a given state and prev_succ_fail if it begins a selector, clearly
# it could be a subset of the whole sequence, but hopefully this problem will
# disappear after simplifying a finite number of times
def check_state_sel(q_check, prev_action_out):
    sel_action = []
    sel_state  = []
    q_f        = []
    [first_action, first_state] = check_selector(q_check, sel_action, sel_state,
                                                 q_f    , prev_action_out)
    if len(sel_action) == 0:
        # print "bye"
        return
    sel_action.insert(0, first_action)
    sel_state.insert (0, first_state )
    comp_action = ' '.join(sel_action)
    comp_action = '{' + comp_action + '}'

    # print "sel_action, sel_state, q_f", sel_action, sel_state, q_f
    # print comp_action

    for idx, q in enumerate(sel_state):
        if idx == 0:            # checkear que pasa cuando no hay ninguno
            print "replacing action", d_action[q-1][prev_action_out], \
                "with", comp_action, "in state", q
            d_transi[q-1][prev_action_out] = q_f[0]
            d_action[q-1][prev_action_out] = comp_action
        else:
            d_transi[q-1][:] = [None, None]
            d_action[q-1][:] = [None, None]

    print "d_transi from check_state_sel", d_transi
    print "d_action from check_state_sel", d_action


################################################################################
# high level functions
################################################################################

# loops through all the states which are not None (simplified) and uses the
# function check_state_seq to modify the global d_transi and d_action. It may
# require several interleaved passes of parse_pass_seq and parse_pass_sel to
# reach the minimum expression, n_times < depth(BT)
def parse_pass_seq(rev):
    if rev:
        looping_states = reversed(states)
    else:
        looping_states = states
    for q_check in looping_states:
        print "checking q:", q_check, "in sequence pass"
        # print d_transi[q_check-1][:]
        # print d_action[q_check-1][:]
        if (d_transi[q_check-1][:] == [None, None] and
            d_action[q_check-1][:] == [None, None]):
            # print "passing", q_check
            continue
        # case = 0: diff q, diff a, case = 1: same q, diff a
        # case = 1: diff q, same a, case = 3: same q, same a
        case = 0
        if (d_transi[q_check-1][1] == d_transi[q_check-1][0] and
            d_action[q_check-1][1] != d_action[q_check-1][0]):
            case = 1
        if (d_transi[q_check-1][1] != d_transi[q_check-1][0] and
            d_action[q_check-1][1] == d_action[q_check-1][0]):
            case = 2
        if (d_transi[q_check-1][1] == d_transi[q_check-1][0] and
            d_action[q_check-1][1] == d_action[q_check-1][0]):
            case = 3
        # print "case", case

        orig_action = d_action[q_check-1][:]
        check_state_seq(q_check, 0) # q_check: 1 -> n

        if   case == 0:
            check_state_seq(q_check, 1)
        elif case == 1:
            action_str  = d_action[q_check-1][0]
            d_action[q_check-1][1] = action_str.replace(orig_action[0],
                                                        orig_action[1], 1)
            d_transi[q_check-1][1] = d_transi[q_check-1][0]
        elif case == 2:
            # does not matter that the action is the same because it
            # goes to a different q, which needs to be analyzed
            check_state_seq(q_check, 1)
        elif case == 3:
            d_action[q_check-1][1] = d_action[q_check-1][0]
            d_transi[q_check-1][1] = d_transi[q_check-1][0]
        # raw_input()
    print "d_transi from parse_pass_seq", d_transi
    print "d_action from parse_pass_seq", d_action

# loops through all the states which are not None (simplified) and uses the
# function check_state_sel to modify the global d_transi and d_action. It may
# require several interleaved passes of parse_pass_seq and parse_pass_sel to
# reach the minimum expression, n_times < depth(BT)
def parse_pass_sel(rev):
    if rev:
        looping_states = reversed(states)
    else:
        looping_states = states
    for q_check in looping_states:
        print "checking q:", q_check, "in selector pass"
        # print d_transi[q_check-1][:]
        # print d_action[q_check-1][:]
        if (d_transi[q_check-1][:] == [None, None] and
            d_action[q_check-1][:] == [None, None]):
            # print "passing", q_check
            continue
        # case = 0: diff q, diff a, case = 1: same q, diff a
        # case = 1: diff q, same a, case = 3: same q, same a
        case = 0
        if (d_transi[q_check-1][1] == d_transi[q_check-1][0] and
            d_action[q_check-1][1] != d_action[q_check-1][0]):
            case = 1
        if (d_transi[q_check-1][1] != d_transi[q_check-1][0] and
            d_action[q_check-1][1] == d_action[q_check-1][0]):
            case = 2
        if (d_transi[q_check-1][1] == d_transi[q_check-1][0] and
            d_action[q_check-1][1] == d_action[q_check-1][0]):
            case = 3
        # print "case", case

        orig_action = d_action[q_check-1][:]
        check_state_sel(q_check, 0) # q_check: 1 -> n

        if   case == 0:
            check_state_sel(q_check, 1)
        elif case == 1:
            action_str  = d_action[q_check-1][0]
            d_action[q_check-1][1] = action_str.replace(orig_action[0],
                                                        orig_action[1], 1)
            d_transi[q_check-1][1] = d_transi[q_check-1][0]
        elif case == 2:
            # does not matter that the action is the same because it
            # goes to a different q, which needs to be analyzed
            check_state_sel(q_check, 1)
        elif case == 3:
            d_action[q_check-1][1] = d_action[q_check-1][0]
            d_transi[q_check-1][1] = d_transi[q_check-1][0]
        # raw_input()
    print "d_transi from parse_pass_sel", d_transi
    print "d_action from parse_pass_sel", d_action

# passes with selector and sequence simplification methods until the number
# of simplified states represented by 'None' ceases to increase
def parse_io_to_bt(reverse_states):
    prev_number_none = -1
    number_none      =  0
    while number_none > prev_number_none:
        prev_number_none = number_none

        if order_seq_sel == True:
            parse_pass_seq(reverse_states)
            parse_pass_sel(reverse_states)
        else:
            parse_pass_sel(reverse_states)
            parse_pass_seq(reverse_states)

        number_none = 0
        for i in range(n_states):
            if d_transi[i] == [None, None] and \
               d_action[i] == [None, None]:
                number_none += 1
        print "number simplified states", number_none, "-------"
    print "exiting because cannot parse any further",  "+++++++"

################################################################################
# simplifying functions
################################################################################

# loop through the list parsed using [ and ] as pairing symbols, treat { and }
# as symbols that prevent what is inside from being put outside (simplified)
def simp_pass_seq(list_parsed):
    verbatim_count = 0
    parsing_string = ''
    list_out = []
    for i, e in enumerate(list_parsed):
        # print "++++++++++++++++++++++++++++"
        # print "list", list_parsed
        # print "-> (type/index/value):", type(e), i, "/", len(list_parsed), e
        if e == '{': verbatim_count += 1
        # print "verbatim_count", verbatim_count
        if verbatim_count == 0:
            if isinstance(e, list):
                temp_list_parsed = simp_pass_seq(e)
                for sub_i, sub_e in enumerate(temp_list_parsed):
                    list_out.append(sub_e)
                    # print "appending list", sub_e, "to", list_out
            else:
                list_out.append(e)
                # print "appending char", e, "to", list_out
        if verbatim_count > 0:
            if isinstance(e, list):
                parsing_string += build_list_seq(e)
                # print "increasing parsing_string to", parsing_string
            else:
                parsing_string += (e + ' ')
                # print "increasing parsing_string to", parsing_string
        if e == '}':
            verbatim_count -= 1
            if verbatim_count == 0:
                # print "parsing_string", parsing_string
                parsed_sel      = nest_sel.parseString(parsing_string)
                list_parsed_sel = parsed_sel.asList()
                temp_list = simp_pass_sel(list_parsed_sel[0])
                temp_parsing_string = build_list_sel(temp_list)
                # print "temp_parsing_string", temp_parsing_string
                temp_parsing_string = '[' + temp_parsing_string + ']'
                # sequence simplification
                parsed_seq      = nest_seq.parseString(temp_parsing_string)
                list_parsed_seq = parsed_seq.asList()
                for new_i, new_e in enumerate(list_parsed_seq[0]):
                    list_out.append(new_e)
                    # print "appending (char/list)", e, "to", list_out
                parsing_string = ''
        # print "----------------------------"
        # raw_input()
    return list_out

# receives a simplified list done using simple_pass_seq and builds the string
# that can be used by simp_pass_seq to give another pass trying to simplify more
def build_list_seq(list_parsed):
    s = '[' # string to be returned with simplified notation
    for i, e in enumerate(list_parsed):
        if isinstance(e, list):
            s += build_list_seq(e)
        elif e == '{':
            s += e
        elif e == '}':
            s = s.rstrip()
            s += (e + ' ')
        else:
            s += (e + ' ')
    s = s.rstrip()
    s += '] '
    return s

# loop through the list parsed using { and } as pairing symbols, treat [ and ]
# as symbols that prevent what is inside from being put outside (simplified)
def simp_pass_sel(list_parsed):
    verbatim_count = 0
    parsing_string = ''
    list_out = []
    for i, e in enumerate(list_parsed):
        # print "++++++++++++++++++++++++++++"
        # print "list", list_parsed
        # print "-> (type/index/value):", type(e), i, "/", len(list_parsed), e
        if e == '[': verbatim_count += 1
        # print "verbatim_count", verbatim_count
        if verbatim_count == 0:
            if isinstance(e, list):
                temp_list_parsed = simp_pass_sel(e)
                for sub_i, sub_e in enumerate(temp_list_parsed):
                    list_out.append(sub_e)
                    # print "appending list", sub_e, "to", list_out
            else:
                list_out.append(e)
                # print "appending char", e, "to", list_out
        if verbatim_count > 0:
            if isinstance(e, list):
                parsing_string += build_list_sel(e)
                # print "increasing parsing_string to", parsing_string
            else:
                parsing_string += (e + ' ')
                # print "increasing parsing_string to", parsing_string
        if e == ']':
            verbatim_count -= 1
            if verbatim_count == 0:
                # print "parsing_string", parsing_string
                parsed_seq      = nest_seq.parseString(parsing_string)
                list_parsed_seq = parsed_seq.asList()
                temp_list = simp_pass_seq(list_parsed_seq[0])
                temp_parsing_string = build_list_seq(temp_list)
                # print "temp_parsing_string", temp_parsing_string
                temp_parsing_string = '{' + temp_parsing_string + '}'
                # selector simplification
                parsed_sel      = nest_sel.parseString(temp_parsing_string)
                list_parsed_sel = parsed_sel.asList()
                for new_i, new_e in enumerate(list_parsed_sel[0]):
                    list_out.append(new_e)
                    # print "appending (char/list)", e, "to", list_out
                parsing_string = ''
        # print "----------------------------"
        # raw_input()
    return list_out

# receives a simplified list done using simple_pass_sel and builds the string
# that can be used by simp_pass_sel to give another pass trying to simplify more
def build_list_sel(list_parsed):
    s = '{' # string to be returned with simplified notation
    for i, e in enumerate(list_parsed):
        if isinstance(e, list):
            s += build_list_sel(e)
        elif e == '[':
            s += e
        elif e == ']':
            s = s.rstrip()
            s += (e + ' ')
        else:
            s += (e + ' ')
    s = s.rstrip()
    s += '} '
    return s

# simplifying the string of the bt consists of simplifying one pass of sequence
# and one pass of selector, and repeating for as long as the string changes
def simplify_tree(parsing_string):
    print "simplifying tree"
    if parsing_string[0] == '[': # sequence
        parsed_seq        = nest_seq.parseString(parsing_string)
        list_parsed_seq   = parsed_seq.asList()
        simplified_list   = simp_pass_seq(list_parsed_seq[0])
        simplified_string = build_list_seq(simplified_list)
    if parsing_string[0] == '{': # selector
        parsed_sel        = nest_sel.parseString(parsing_string)
        list_parsed_sel   = parsed_sel.asList()
        simplified_list   = simp_pass_sel(list_parsed_sel[0])
        simplified_string = build_list_sel(simplified_list)
    if simplified_string != parsing_string:
        print "simplifying again because it improved (prev, after): \n", \
            parsing_string, '\n',  simplified_string
        return simplify_tree(simplified_string)
    else:
        print "exiting because it did not improve anymore"
        return parsing_string
    return simplified_string

# writes the simplified bt string to a file using tab indentation, it needs to
# build first the string with spaces in order to use split
def write_bt_file(bt_string):
    bt_string_spaces = ''
    for char in bt_string:
        print "char", char
        if char == '{' or char == '}' or \
           char == '[' or char == ']':
            bt_string_spaces += ' '+ char + ' '
        else:
            bt_string_spaces += char
    print "bt_string_spaces", bt_string_spaces
    b = open(bt_filename, 'w')
    indent  = ''
    bt_list = bt_string_spaces.split()
    for elem in bt_list:
        print "elem", elem
        if elem == '}' or elem == ']':
            indent = indent[1:]
        b.write(indent + elem + '\n')
        if elem == '{' or elem == '[':
            indent += '\t'
    b.close()


################################
################################
################################
################################
################################
################################
################################
############# main #############
################################
################################
################################
################################
################################
################################


# initial checkups
print "#### PHASE 1: CHECKING I/O AUTOMATA FOR ERRORS ####"
check_initial(q_init)
print "###################################################"

# io automata parse to bt
print "#### PHASE 2: PARSING I/O AUTOMATA TO BEHAVIOR TREE ####"
reverse_states = True
order_seq_sel  = True
# order_seq_sel  = False
# reverse_states = False
parse_io_to_bt(reverse_states)
parsed_bt_string = check_parse_result(q_init)
print "########################################################"

# bt simplification
print "#### PHASE 3: SIMPLIFYING BEHAVIOR TREE STRING ####"
parsing_string   = parsed_bt_string
# parsing_string = "[1 [2 [3 [4 [5 6]]]] [[7 {8 [9 10] 11} 12] 13 14] 15]"
# parsing_string = "{1 {2 {3 {4 {5 6}}}} {{7 [8 {9 10} 11] 12} 13 14} 15}"
print "parsing string", parsing_string
simple_bt = simplify_tree(parsing_string)
print "final behavior tree", simple_bt
write_bt_file(simple_bt)
print "###################################################"
