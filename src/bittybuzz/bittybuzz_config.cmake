set_default(BBZHEAP_SIZE 16384)
set_default(BBZHEAP_ELEMS_PER_TSEG 5)
set_default(BBZSTACK_SIZE 1024)
set_default(BBZVSTIG_CAP 4)
set_default(BBZNEIGHBORS_CAP 15)
set_default(BBZINMSG_QUEUE_CAP 32)
set_default(BBZOUTMSG_QUEUE_CAP 32)
set_default(BBZHEAP_RSV_ACTREC_MAX 28)
set_default(BBZLAMPORT_THRESHOLD 50)
set_default(BBZHEAP_GCMARK_DEPTH 8)
set_default(BBZMSG_IN_PROC_MAX 10)
set_default(BBZNEIGHBORS_CLR_PERIOD 10)
set_default(BBZNEIGHBORS_MARK_TIME 4)


option(BBZ_XTREME_MEMORY "Enables high memory-optimization." OFF)
option(BBZ_USE_PRIORITY_SORT "Enables the use of priority sort on out-messages queue." OFF)
option(BBZ_USE_FLOAT "Enables float type." ON)
option(BBZ_DISABLE_NEIGHBORS "Disables the neighbors data structure and messages." OFF)
option(BBZ_DISABLE_VSTIGS "Disables the virtual stigmergy data structure and messages." OFF)
option(BBZ_DISABLE_SWARMS "Disables swarms data structures and messages." OFF)
option(BBZ_DISABLE_MESSAGES "Disables usage and transfer of any kind of Buzz message." OFF)
option(BBZ_BYTEWISE_ASSIGNMENT "Enables byte per byte assignment." OFF)
option(BBZ_NEIGHBORS_USE_FLOATS "Whether to use floats for the neighbor's range and bearing measurments." ON)
option(BBZ_ENABLE_FLOAT_OPERATIONS "Whether to enable floats operations" ON)

# There is no implementation of the swarmlist broadcast on bittybuzz for now
option(BBZ_DISABLE_SWARMLIST_BROADCASTS "Whether we disable the broadcasting of our swarmlist to neighboring robots." ON)

# TODO: Check if we need it, I don't think we will compile the code in this project so the flag may not be necessary
option(BBZ_DISABLE_PY_BEHAV "Disables Python behaviors of closures (make closure behave like in JavaScript)." OFF)


# Swarmus BBZ Options
set_default(BBZ_CLOSURE_REGISTER_LENGTH 32)
