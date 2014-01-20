io_filename = "sampleIO.txt"
bt_filename = "sampleBT.txt"
f = open(io_filename, 'r')
b = open(bt_filename, 'w')

n_states = int(f.readline())
q_init = int(f.readline())
sigma_in  = range(0, 2)         # 0: fail, 1: succ
sigma_out = range(1, int(f.readline()) + 1 )
d_transi  = [[ None for x in range(2)] for y in range(n_states)]
d_action  = [[ None for x in range(2)] for y in range(n_states)]

for i in range(n_states):
    d_transi[i] = map(int, f.readline().split())
for i in range(n_states):
    d_action[i] = map(int, f.readline().split())

print n_states
print q_init
print sigma_in
print sigma_out
print d_transi
print d_action

b.write('{\n')
# b.write('\n')

for i in range(n_states):
    b.write('\t[\n')                         #
    b.write('\t\tC Q = ' + str(i+1) + '\n')  ##

    b.write('\t\t{\n')                       ##

    b.write('\t\t\t[\n')                     ###
    b.write('\t\t\t\tC yes\n')               ####
    b.write('\t\t\t\td '   + str(d_transi[i][0]) + '\n')    ####
    b.write('\t\t\t\t\tA ' + str(d_action[i][0]) + '\n')    #####
    b.write('\t\t\t\tD\n')                                  ####
    b.write('\t\t\t]\n')                     ###

    b.write('\t\t\t[\n')                     ###
    b.write('\t\t\t\tC no\n' )               ####
    b.write('\t\t\t\td '   + str(d_transi[i][1]) + '\n')    ####
    b.write('\t\t\t\t\tA ' + str(d_action[i][1]) + '\n')    #####
    b.write('\t\t\t\tD\n')                                  ####
    b.write('\t\t\t]\n')                     ###


    b.write('\t\t}\n')                       ##

    b.write('\t]\n')                         #


b.write('}\n')

# b.write("hola")
# b.write("hola")
# b.write("hola")
b.close()
