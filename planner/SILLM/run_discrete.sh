set -ex
./compile.sh

EXEC_FP=build/discrete_driver

MAP_FP="../../data/map/small/sortation_small.map"
NUM_AGENTS=600
RANDOM_SEED=0
SIM_STEPS=512
# PLAN_TIME_LIMIT will be ignored if LNS_MAX_ITER is enabled
# in seconds
PLAN_TIME_LIMIT=1

LNS_PLAN_WINDOW=15
LNS_EXEC_WINDOW=1
LNS_NUM_THREADS=1
# if set to -1, it will be ignored and use PLAN_TIME_LIMIT
LNS_MAX_ITER=5000

# gdb --args \
./${EXEC_FP} --map_fp ${MAP_FP} --num_agents ${NUM_AGENTS} --random_seed ${RANDOM_SEED} \
    --sim_steps ${SIM_STEPS} --plan_time_limit ${PLAN_TIME_LIMIT} \
    --lns_plan_window ${LNS_PLAN_WINDOW} --lns_exec_window ${LNS_EXEC_WINDOW} \
    --lns_num_threads ${LNS_NUM_THREADS} --lns_max_iters ${LNS_MAX_ITER}